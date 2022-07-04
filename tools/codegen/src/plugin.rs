//! Codegen for openrr-plugin

use std::{mem, path::Path};

use anyhow::Result;
use fs_err as fs;
use heck::ToSnakeCase;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use syn::{
    parse_quote,
    visit_mut::{self, VisitMut},
    Ident,
};

use super::*;

pub fn gen(workspace_root: &Path) -> Result<()> {
    const FULLY_IGNORE: &[&str] = &["SetCompleteCondition"];
    const IGNORE: &[&str] = &["JointTrajectoryClient", "SetCompleteCondition", "Gamepad"];
    const USE_TRY_INTO: &[&str] = &["SystemTime"];

    let out_dir = &workspace_root.join("openrr-plugin/src/gen");
    fs::create_dir_all(out_dir)?;
    let mut api_items = TokenStream::new();
    let mut proxy_impls = TokenStream::new();
    let mut traits = vec![];
    for item in arci_traits(workspace_root)? {
        let name = &&*item.ident.to_string();
        if FULLY_IGNORE.contains(name) {
            continue;
        }
        traits.push(item.ident.clone());
        if IGNORE.contains(name) {
            continue;
        }

        let trait_name = &item.ident;
        let proxy_name = format_ident!("{trait_name}Proxy");
        let proxy_name_lit = proxy_name.to_string();
        let trait_object_name = format_ident!("{trait_name}TraitObject");
        let methods = item.items.iter().map(|method| match method {
            syn::TraitItem::Method(method) => {
                let sig = &method.sig;
                let name = &sig.ident;
                let args = sig.inputs.iter().map(|arg| match arg {
                    syn::FnArg::Receiver(_) => quote! {},
                    syn::FnArg::Typed(arg) => {
                        let pat = &arg.pat;
                        if let Some(path) = get_ty_path(&arg.ty) {
                            if USE_TRY_INTO
                                .contains(&&*path.segments.last().unwrap().ident.to_string())
                            {
                                return quote! { #pat.try_into()?, };
                            }
                        }
                        if matches!(&*arg.ty, syn::Type::Reference(_)) && !is_str(&arg.ty) {
                            return quote! { (*#pat).into(), };
                        }
                        // TODO: handle Vec
                        quote! { #pat.into(), }
                    }
                });
                quote! {
                    #sig {
                        Ok(self.0.#name(#(#args)*).into_result()?.into())
                    }
                }
            }
            _ => quote! {},
        });
        let proxy_type = gen_proxy_type(trait_name, quote! { arci:: });
        api_items.extend(quote! {
            #proxy_type

            impl arci::#trait_name for #proxy_name {
                #(#methods)*
            }

            impl std::fmt::Debug for #proxy_name {
                fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    f.debug_struct(#proxy_name_lit).finish()
                }
            }
        });

        let sabi_trait_name = format_ident!("R{trait_name}Trait");
        let sabi_trait_trait_object_name = format_ident!("{sabi_trait_name}_TO");
        let mut sabi_method_def = vec![];
        let mut sabi_method_impl = vec![];
        for method in &item.items {
            if let syn::TraitItem::Method(method) = method {
                struct ReplacePath;
                impl VisitMut for ReplacePath {
                    fn visit_type_mut(&mut self, ty: &mut syn::Type) {
                        if is_str(ty) {
                            *ty = parse_quote!(RStr<'_>);
                            return;
                        }
                        if let syn::Type::Reference(t) = ty {
                            *ty =
                                mem::replace(&mut *t.elem, syn::Type::Verbatim(TokenStream::new()));
                        }

                        visit_mut::visit_type_mut(self, ty);
                    }

                    fn visit_path_mut(&mut self, path: &mut syn::Path) {
                        let mut last = path.segments.pop().unwrap().into_value();
                        if last.ident.to_string().starts_with("Isometry") {
                            last.arguments = syn::PathArguments::None;
                            last.ident = format_ident!("{}F64", last.ident);
                        } else if last.ident == "WaitFuture" {
                            last.ident = format_ident!("BlockingWait");
                        }
                        last.ident = format_ident!("R{}", last.ident);
                        path.segments.clear();
                        path.segments.push(last);
                        visit_mut::visit_path_mut(self, path);
                    }
                }

                let method_name = &method.sig.ident;
                let args = method.sig.inputs.iter().map(|arg| match arg {
                    syn::FnArg::Receiver(_) => quote! { self },
                    syn::FnArg::Typed(arg) => {
                        let pat = &arg.pat;
                        if let Some(path) = get_ty_path(&arg.ty) {
                            if USE_TRY_INTO
                                .contains(&&*path.segments.last().unwrap().ident.to_string())
                            {
                                return quote! { rtry!(#pat.try_into()) };
                            }
                        }
                        if matches!(&*arg.ty, syn::Type::Reference(_)) && !is_str(&arg.ty) {
                            return quote! { &#pat.into() };
                        }
                        // TODO: handle Vec
                        quote! { #pat.into() }
                    }
                });
                let mut sig = method.sig.clone();
                ReplacePath.visit_signature_mut(&mut sig);
                sabi_method_def.push(quote! {
                    #sig;
                });
                sabi_method_impl.push(quote! {
                    #sig {
                        ROk(rtry!(arci::#trait_name::#method_name(#(#args),*)).into())
                    }
                })
            }
        }
        proxy_impls.extend(quote! {
            pub(crate) type #trait_object_name = #sabi_trait_trait_object_name<RBox<()>>;

            #[abi_stable::sabi_trait]
            pub(crate) trait #sabi_trait_name: Send + Sync + 'static {
                #(#sabi_method_def)*
            }

            impl<T> #sabi_trait_name for T
            where
                T: arci::#trait_name + 'static
            {
                #(#sabi_method_impl)*
            }
        });
    }

    let (plugin_trait_api, plugin_trait_proxy) = gen_plugin_trait(&traits);
    let api = quote! {
        use arci::{
            BaseVelocity,
            Error,
            Isometry2,
            Isometry3,
            WaitFuture,
        };
        use abi_stable::StableAbi;
        use super::*;
        #plugin_trait_api
        #api_items
    };
    let proxy = quote! {
        use abi_stable::{
            rtry,
            std_types::{RBox, RDuration, ROk, RResult, RStr},
        };
        use super::*;
        #plugin_trait_proxy
        #proxy_impls
    };

    write(&out_dir.join("api.rs"), api)?;
    write(&out_dir.join("proxy.rs"), proxy)?;
    Ok(())
}

fn gen_plugin_trait(traits: &[Ident]) -> (TokenStream, TokenStream) {
    let mut plugin_method_def = vec![];
    let mut plugin_method_impl = vec![];
    let mut sabi_plugin_method_def = vec![];
    let mut sabi_plugin_method_impl = vec![];
    for trait_name in traits {
        let method_name = format_ident!("new_{}", trait_name.to_string().to_snake_case());
        let proxy_name = format_ident!("{trait_name}Proxy");
        let new_doc = format!(
            " Creates a new instance of [`arci::{trait_name}`] with the specified arguments.",
        );
        plugin_method_def.push(quote! {
            #[doc = #new_doc]
            fn #method_name(
                &self,
                args: String,
            ) -> Result<Option<Box<dyn arci::#trait_name>>, arci::Error> {
                let _ = args;
                Ok(None)
            }
        });
        plugin_method_impl.push(quote! {
            #[doc = #new_doc]
            pub fn #method_name(
                &self,
                args: String,
            ) -> Result<Option<#proxy_name>, arci::Error> {
                Ok(self.0.#method_name(args.into()).into_result()?.into_option())
            }
        });

        let sabi_method_sig = quote! {
            fn #method_name(
                &self,
                args: RString,
            ) -> RResult<ROption<crate::#proxy_name>, RError>
        };
        sabi_plugin_method_def.push(quote! {
            #sabi_method_sig;
        });
        sabi_plugin_method_impl.push(quote! {
            #sabi_method_sig {
                ROk(rtry!(crate::Plugin::#method_name(self, args.into()))
                    .map(crate::#proxy_name::new)
                    .into())
            }
        });
    }
    let proxy_type = gen_proxy_type(&format_ident!("Plugin"), quote! {});
    let api = quote! {
        /// The plugin trait.
        pub trait Plugin: Send + Sync + 'static {
            #(#plugin_method_def)*
        }
        #proxy_type
        impl PluginProxy {
            #(#plugin_method_impl)*
        }
    };
    let proxy = quote! {
        pub(crate) type PluginTraitObject = RPluginTrait_TO<RBox<()>>;

        #[sabi_trait]
        pub(crate) trait RPluginTrait: Send + Sync + 'static {
            #(#sabi_plugin_method_def)*
        }

        impl<T> RPluginTrait for T
        where
            T: crate::Plugin,
        {
            #(#sabi_plugin_method_impl)*
        }
    };
    (api, proxy)
}

fn gen_proxy_type(trait_name: &Ident, prefix_path: TokenStream) -> TokenStream {
    let proxy_name = format_ident!("{trait_name}Proxy");
    let trait_object_name = format_ident!("{trait_name}TraitObject");
    let new_doc = format!(" Creates a new `{proxy_name}`.");
    let struct_doc = format!(
        " FFI-safe equivalent of [`Box<dyn {0}{trait_name}>`]({0}{trait_name}).",
        prefix_path.to_string().replace(' ', ""),
    );
    quote! {
        #[doc = #struct_doc]
        #[derive(StableAbi)]
        #[repr(C)]
        pub struct #proxy_name(pub(crate) crate::proxy::#trait_object_name);

        impl #proxy_name {
            #[doc = #new_doc]
            pub fn new<T>(inner: T) -> Self
            where
                T: #prefix_path #trait_name + 'static
            {
                Self(crate::proxy::#trait_object_name::from_value(
                    inner,
                    abi_stable::erased_types::TD_Opaque,
                ))
            }
        }
    }
}
