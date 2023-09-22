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
    const IGNORE: &[&str] = &["SetCompleteCondition"];
    const USE_TRY_INTO: &[&str] = &["SystemTime"];

    let out_dir = &workspace_root.join("openrr-plugin/src/gen");
    fs::create_dir_all(out_dir)?;
    let mut api_items = TokenStream::new();
    let mut proxy_impls = TokenStream::new();
    let mut trait_names = vec![];
    let (arci_traits, arci_structs, arci_enums) = arci_types(workspace_root)?;
    for item in arci_traits {
        let name = &&*item.ident.to_string();
        if FULLY_IGNORE.contains(name) {
            continue;
        }
        trait_names.push(item.ident.clone());
        if IGNORE.contains(name) {
            continue;
        }

        let trait_name = &item.ident;
        let mut is_async_trait = false;
        let mut arci_method_impl = vec![];
        let mut sabi_method_def = vec![];
        let mut sabi_method_impl = vec![];
        for method in &item.items {
            if let syn::TraitItem::Fn(method) = method {
                let sig = &method.sig;
                let method_name = &sig.ident;
                let is_async_method = sig.asyncness.is_some();
                if is_async_method {
                    is_async_trait = true;
                }
                let mut has_receiver = false;
                let mut arci_args = vec![];
                let mut sabi_args = vec![];
                for arg in &sig.inputs {
                    match arg {
                        syn::FnArg::Receiver(_) => {
                            has_receiver = true;
                            sabi_args.push(if is_async_trait {
                                if is_async_method {
                                    quote! { &*this }
                                } else {
                                    quote! { &**self }
                                }
                            } else {
                                quote! { self }
                            });
                        }
                        syn::FnArg::Typed(arg) => {
                            let pat = &arg.pat;
                            if let Some(path) = get_ty_path(&arg.ty) {
                                if USE_TRY_INTO
                                    .contains(&&*path.segments.last().unwrap().ident.to_string())
                                {
                                    arci_args.push(quote! { #pat.try_into()? });
                                    sabi_args.push(quote! { rtry!(#pat.try_into()) });
                                    continue;
                                }
                            }
                            if matches!(&*arg.ty, syn::Type::Reference(_)) && !is_str(&arg.ty) {
                                arci_args.push(quote! { (*#pat).into() });
                                sabi_args.push(quote! { &#pat.into() });
                                continue;
                            }
                            let t = if is_vec(&arg.ty).map_or(false, |ty| !is_primitive(ty)) {
                                quote! { #pat.into_iter().map(Into::into).collect() }
                            } else {
                                quote! { #pat.into() }
                            };
                            arci_args.push(t.clone());
                            sabi_args.push(t);
                        }
                    }
                }
                let (return_result, is_vec) = match &method.sig.output {
                    syn::ReturnType::Type(_, ty) => match is_result(ty) {
                        Some(ty) => (true, is_vec(ty).map_or(false, |ty| !is_primitive(ty))),
                        None => (false, is_vec(ty).map_or(false, |ty| !is_primitive(ty))),
                    },
                    syn::ReturnType::Default => (false, false),
                };
                let into = if is_vec {
                    quote! { .into_iter().map(Into::into).collect() }
                } else {
                    quote! { .into() }
                };
                {
                    // impl arci::<trait_name> for <trait_name>Proxy
                    let mut call = quote! { self.0.#method_name(#(#arci_args),*) };
                    if is_async_method {
                        call = quote! { #call.await };
                    }
                    call = if return_result {
                        quote! { Ok(#call.into_result()?#into) }
                    } else {
                        quote! { #call #into }
                    };
                    arci_method_impl.push(quote! {
                        #sig { #call }
                    });
                }
                {
                    // impl R<trait_name>Trait for (impl arci::<trait_name>)
                    let mut sig = sig.clone();
                    sig.asyncness = None;
                    ReplacePath.visit_signature_mut(&mut sig);
                    if is_async_method {
                        sig.output = match &sig.output {
                            syn::ReturnType::Type(_, ty) => parse_quote! { -> FfiFuture<#ty> },
                            syn::ReturnType::Default => parse_quote! { -> FfiFuture<()> },
                        };
                    }
                    sabi_method_def.push(quote! { #sig; });
                    let mut call = quote! { arci::#trait_name::#method_name(#(#sabi_args),*) };
                    if is_async_method {
                        call = quote! { #call.await };
                    }
                    call = if return_result {
                        quote! { ROk(rtry!(#call)#into) }
                    } else {
                        quote! { #call #into }
                    };
                    if is_async_method {
                        call = quote! { async move { #call }.into_ffi() };
                        if has_receiver {
                            call = quote! {
                                let this = self.clone();
                                #call
                            };
                        }
                    }
                    sabi_method_impl.push(quote! {
                        #sig {
                            let _guard = crate::TOKIO.enter();
                            #call
                        }
                    });
                }
            }
        }

        // impl arci::<trait_name> for <trait_name>Proxy
        let proxy_name = format_ident!("{trait_name}Proxy");
        let proxy_name_lit = proxy_name.to_string();
        let proxy_type = gen_proxy_type(trait_name, quote! { arci:: }, is_async_trait);
        let async_trait = if is_async_trait {
            quote! { #[arci::async_trait] }
        } else {
            quote! {}
        };
        api_items.extend(quote! {
            #proxy_type
            #async_trait
            impl arci::#trait_name for #proxy_name {
                #(#arci_method_impl)*
            }
            impl std::fmt::Debug for #proxy_name {
                fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    f.debug_struct(#proxy_name_lit).finish()
                }
            }
        });

        // impl R<trait_name>Trait for (impl arci::<trait_name>)
        let trait_object_name = format_ident!("{trait_name}TraitObject");
        let sabi_trait_name = format_ident!("R{trait_name}Trait");
        let sabi_trait_trait_object_name = format_ident!("{sabi_trait_name}_TO");
        let this = if is_async_trait {
            quote! { Arc<T> }
        } else {
            quote! { T }
        };
        let clone = if is_async_trait {
            quote! { Clone + }
        } else {
            quote! {}
        };
        let sized = if is_async_trait {
            quote! { ?Sized + }
        } else {
            quote! {}
        };
        proxy_impls.extend(quote! {
            pub(crate) type #trait_object_name = #sabi_trait_trait_object_name<RBox<()>>;
            #[abi_stable::sabi_trait]
            pub(crate) trait #sabi_trait_name: Send + Sync + #clone 'static {
                #(#sabi_method_def)*
            }
            impl<T> #sabi_trait_name for #this
            where
                T: #sized arci::#trait_name + 'static
            {
                #(#sabi_method_impl)*
            }
        });
    }
    fn map_field(
        pats: &mut Vec<TokenStream>,
        from_arci: &mut Vec<TokenStream>,
        to_arci: &mut Vec<TokenStream>,
        syn::Field { ident, ty, .. }: &syn::Field,
        index: usize,
    ) {
        let index_or_ident = ident
            .clone()
            .unwrap_or_else(|| format_ident!("field{}", index));
        let pat = quote! { #index_or_ident, };
        pats.push(pat.clone());
        if is_primitive(ty) {
            from_arci.push(pat.clone());
            to_arci.push(pat);
            return;
        }
        if let Some(ty) = is_option(ty) {
            if let Some(ty) = is_vec(ty) {
                let t = if is_primitive(ty) {
                    quote! { map(|v| v.into_iter().collect()) }
                } else {
                    quote! { map(|v| v.into_iter().map(Into::into).collect()) }
                };
                let mut to = quote! { #index_or_ident.into_option().#t, };
                let mut from = quote! { #index_or_ident.#t.into(), };
                if ident.is_some() {
                    from = quote! { #ident: #from };
                    to = quote! { #ident: #to };
                }
                from_arci.push(from);
                to_arci.push(to);
                return;
            }
        }
        if let Some(ty) = is_vec(ty) {
            let mut t = if is_primitive(ty) {
                quote! { #index_or_ident.into_iter().collect(), }
            } else {
                quote! { #index_or_ident.into_iter().map(Into::into).collect(), }
            };
            if ident.is_some() {
                t = quote! { #ident: #t };
            }
            from_arci.push(t.clone());
            to_arci.push(t);
            return;
        }
        let mut t = quote! { #index_or_ident.into(), };
        if ident.is_some() {
            t = quote! { #ident: #t };
        }
        from_arci.push(t.clone());
        to_arci.push(t);
    }
    // Generate R* structs.
    for item in arci_structs {
        let arci_name = &item.ident;
        let arci_path = quote! { arci::#arci_name };
        let r_name = format_ident!("R{}", item.ident);
        let struct_doc = format!(" FFI-safe equivalent of [`arci::{arci_name}`].");
        let fields = item
            .fields
            .iter()
            .cloned()
            .map(|syn::Field { ident, mut ty, .. }| {
                ReplacePath.visit_type_mut(&mut ty);
                quote! { #ident: #ty, }
            });
        let mut pats = vec![];
        let mut from_arci = vec![];
        let mut to_arci = vec![];
        for (i, field) in item.fields.iter().enumerate() {
            map_field(&mut pats, &mut from_arci, &mut to_arci, field, i);
        }
        proxy_impls.extend(quote! {
            #[doc = #struct_doc]
            #[derive(StableAbi)]
            #[repr(C)]
            pub(crate) struct #r_name {
                #(#fields)*
            }
            impl From<#arci_path> for #r_name {
                fn from(v: #arci_path) -> Self {
                    let #arci_path { #(#pats)* } = v;
                    Self { #(#from_arci)* }
                }
            }
            impl From<#r_name> for #arci_path {
                fn from(v: #r_name) -> Self {
                    let #r_name { #(#pats)* } = v;
                    Self { #(#to_arci)* }
                }
            }
        });
    }
    // Generate R* enums.
    for item in arci_enums {
        let arci_name = &item.ident;
        let arci_path = match arci_name.to_string().as_str() {
            // TODO: auto-determine current module
            "Button" | "Axis" | "GamepadEvent" => quote! { arci::gamepad::#arci_name },
            _ => quote! { arci::#arci_name },
        };
        let r_name = format_ident!("R{}", item.ident);
        let enum_doc = format!(" FFI-safe equivalent of [`arci::{arci_name}`].");
        let mut variants = vec![];
        let mut from_arci = vec![];
        let mut to_arci = vec![];
        for v in &item.variants {
            let mut pats_fields = vec![];
            let mut from_arci_fields = vec![];
            let mut to_arci_fields = vec![];
            for (i, field) in v.fields.iter().enumerate() {
                map_field(
                    &mut pats_fields,
                    &mut from_arci_fields,
                    &mut to_arci_fields,
                    field,
                    i,
                );
            }
            match &v.fields {
                syn::Fields::Named(..) => {
                    let fields =
                        v.fields
                            .iter()
                            .cloned()
                            .map(|syn::Field { ident, mut ty, .. }| {
                                ReplacePath.visit_type_mut(&mut ty);
                                quote! { #ident: #ty, }
                            });
                    let ident = &v.ident;
                    variants.extend(quote! { #ident { #(#fields)* }, });
                    from_arci.extend(
                        quote! { #arci_path::#ident { #(#pats_fields)* } => Self::#ident { #(#from_arci_fields)* }, },
                    );
                    to_arci.extend(
                        quote! { #r_name::#ident { #(#pats_fields)* } => Self::#ident { #(#to_arci_fields)* }, },
                    );
                }
                syn::Fields::Unnamed(..) => {
                    let fields = v.fields.iter().cloned().map(|syn::Field { mut ty, .. }| {
                        ReplacePath.visit_type_mut(&mut ty);
                        quote! { #ty, }
                    });
                    let ident = &v.ident;
                    variants.extend(quote! { #ident(#(#fields)*), });
                    from_arci.extend(
                        quote! { #arci_path::#ident( #(#pats_fields)* ) => Self::#ident( #(#from_arci_fields)* ), },
                    );
                    to_arci.extend(
                        quote! { #r_name::#ident( #(#pats_fields)* ) => Self::#ident( #(#to_arci_fields)* ), },
                    );
                }
                syn::Fields::Unit => {
                    let ident = &v.ident;
                    variants.extend(quote! { #ident, });
                    from_arci.extend(quote! { #arci_path::#ident => Self::#ident, });
                    to_arci.extend(quote! { #r_name::#ident => Self::#ident, });
                }
            }
        }
        proxy_impls.extend(quote! {
            #[doc = #enum_doc]
            #[derive(StableAbi)]
            #[repr(C)]
            pub(crate) enum #r_name {
                #(#variants)*
            }
            impl From<#arci_path> for #r_name {
                fn from(v: #arci_path) -> Self {
                    match v {
                        #(#from_arci)*
                    }
                }
            }
            impl From<#r_name> for #arci_path {
                fn from(v: #r_name) -> Self {
                    match v {
                        #(#to_arci)*
                    }
                }
            }
        });
    }

    let (plugin_trait_api, plugin_trait_proxy) = gen_plugin_trait(&trait_names);
    let api = quote! {
        // TODO: remove the need for `arci::` imports.
        use abi_stable::StableAbi;
        use arci::{
            gamepad::GamepadEvent,
            BaseVelocity,
            Error,
            Isometry2,
            Isometry3,
            Scan2D,
            TrajectoryPoint,
            WaitFuture,
        };
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

struct ReplacePath;
impl VisitMut for ReplacePath {
    fn visit_type_mut(&mut self, ty: &mut syn::Type) {
        if is_primitive(ty) {
            return;
        }
        if is_str(ty) {
            *ty = parse_quote!(RStr<'_>);
            return;
        }
        if let syn::Type::Reference(t) = ty {
            *ty = mem::replace(&mut *t.elem, syn::Type::Verbatim(TokenStream::new()));
        }

        visit_mut::visit_type_mut(self, ty);
    }

    fn visit_path_mut(&mut self, path: &mut syn::Path) {
        let mut last = path.segments.pop().unwrap().into_value();
        if last.ident.to_string().starts_with("Isometry") {
            last.arguments = syn::PathArguments::None;
            last.ident = format_ident!("{}F64", last.ident);
        }
        last.ident = format_ident!("R{}", last.ident);
        path.segments.clear();
        path.segments.push(last);
        visit_mut::visit_path_mut(self, path);
    }

    fn visit_fn_arg_mut(&mut self, arg: &mut syn::FnArg) {
        match arg {
            syn::FnArg::Receiver(_) => {}
            syn::FnArg::Typed(arg) => self.visit_pat_type_mut(arg),
        }
    }
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
    let proxy_type = gen_proxy_type(&format_ident!("Plugin"), quote! {}, false);
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

// Generate *Proxy type.
fn gen_proxy_type(
    trait_name: &Ident,
    prefix_path: TokenStream,
    is_async_trait: bool,
) -> TokenStream {
    let proxy_name = format_ident!("{trait_name}Proxy");
    let trait_object_name = format_ident!("{trait_name}TraitObject");
    let new_doc = format!(" Creates a new `{proxy_name}`.");
    let struct_doc = format!(
        " FFI-safe equivalent of [`Box<dyn {0}{trait_name}>`]({0}{trait_name}).",
        prefix_path.to_string().replace(' ', ""),
    );
    // Don't implement Clone even if it is async trait -- use of Arc is implementation detail.
    let inner = if is_async_trait {
        quote! { Arc::new(inner) }
    } else {
        quote! { inner }
    };
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
                    #inner,
                    abi_stable::erased_types::TD_Opaque,
                ))
            }
        }
    }
}
