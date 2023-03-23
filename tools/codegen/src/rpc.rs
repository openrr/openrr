//! Codegen for openrr-remote

use std::path::Path;

use anyhow::Result;
use fs_err as fs;
use heck::ToSnakeCase;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use syn::{
    visit_mut::{self, VisitMut},
    Ident, ItemTrait,
};

use super::*;

pub fn gen(workspace_root: &Path) -> Result<()> {
    const FULLY_IGNORE: &[&str] = &["SetCompleteCondition"];
    const IGNORE: &[&str] = &["JointTrajectoryClient", "SetCompleteCondition", "Gamepad"];

    let out_dir = &workspace_root.join("openrr-remote/src/gen");
    fs::create_dir_all(out_dir)?;
    let mut items = TokenStream::new();
    let mut traits = vec![];

    let mut pb_traits = vec![];
    let pb_file = fs::read_to_string(workspace_root.join("openrr-remote/src/generated/arci.rs"))?;
    CollectTrait(&mut pb_traits).visit_file_mut(&mut syn::parse_file(&pb_file)?);

    for item in arci_traits(workspace_root)? {
        let name = &&*item.ident.to_string();
        if FULLY_IGNORE.contains(name) {
            continue;
        }
        traits.push(item.ident.clone());

        let trait_name = &item.ident;
        items.extend(gen_remote_types(trait_name));

        if IGNORE.contains(name) {
            continue;
        }

        items.extend(gen_client_impl(trait_name, &item));
        items.extend(gen_server_impl(trait_name, &item, &pb_traits));
    }

    let items = quote! {
        use arci::{
            BaseVelocity,
            Error,
            Isometry2,
            Isometry3,
            WaitFuture,
        };
        use super::*;
        #items
    };

    write(&out_dir.join("impls.rs"), items)?;
    Ok(())
}

fn gen_remote_types(trait_name: &Ident) -> TokenStream {
    let client_name = format_ident!("Remote{trait_name}Sender");
    let client_pb_ty = format_ident!("{trait_name}Client");
    let client_pb_mod = format_ident!("{}_client", trait_name.to_string().to_snake_case());
    let server_name = format_ident!("Remote{trait_name}Receiver");
    let server_pb_ty = format_ident!("{trait_name}Server");
    let server_pb_mod = format_ident!("{}_server", trait_name.to_string().to_snake_case());

    quote! {
        #[derive(Debug, Clone)]
        pub struct #client_name {
            pub(crate) client: pb::#client_pb_mod::#client_pb_ty<tonic::transport::Channel>,
        }

        impl #client_name {
            /// Attempt to create a new sender by connecting to a given endpoint.
            pub async fn connect<D>(dst: D) -> Result<Self, arci::Error>
            where
                D: TryInto<tonic::transport::Endpoint>,
                D::Error: Into<Box<dyn std::error::Error + Send + Sync>>,
            {
                let client = pb::#client_pb_mod::#client_pb_ty::connect(dst)
                    .await
                    .map_err(|e| arci::Error::Connection {
                        message: e.to_string(),
                    })?;
                Ok(Self { client })
            }

            /// Create a new sender.
            pub fn new(channel: tonic::transport::Channel) -> Self {
                Self {
                    client: pb::#client_pb_mod::#client_pb_ty::new(channel),
                }
            }
        }

        #[derive(Debug)]
        pub struct #server_name<T> {
            pub(crate) inner: T,
        }

        impl<T> #server_name<T>
        where
            T: arci::#trait_name + 'static,
        {
            /// Create a new receiver.
            pub fn new(inner: T) -> Self {
                Self { inner }
            }

            /// Convert this receiver into a tower service.
            pub fn into_service(self) -> pb::#server_pb_mod::#server_pb_ty<Self> {
                pb::#server_pb_mod::#server_pb_ty::new(self)
            }

            pub async fn serve(self, addr: SocketAddr) -> Result<(), arci::Error> {
                tonic::transport::Server::builder()
                    .add_service(self.into_service())
                    .serve(addr)
                    .await
                    .map_err(|e| arci::Error::Connection {
                        message: e.to_string(),
                    })?;
                Ok(())
            }
        }
    }
}

fn gen_client_impl(trait_name: &Ident, item: &ItemTrait) -> TokenStream {
    let client_name = format_ident!("Remote{trait_name}Sender");

    let methods = item.items.iter().map(|method| match method {
        syn::TraitItem::Fn(method) => {
            let sig = &method.sig;
            let name = &sig.ident;
            let args: Vec<_> = sig
                .inputs
                .iter()
                .filter_map(|arg| match arg {
                    syn::FnArg::Receiver(_) => None,
                    syn::FnArg::Typed(arg) => {
                        let pat = &arg.pat;
                        Some(
                            if matches!(&*arg.ty, syn::Type::Reference(..)) && !is_str(&arg.ty) {
                                quote! { (*#pat) }
                            } else {
                                quote! { #pat }
                            },
                        )
                    }
                })
                .collect();
            let args = match args.len() {
                0 => quote! { () },
                1 => quote! { #(#args)*.into() },
                _ => quote! { (#(#args),*).into() },
            };
            let call = match &sig.output {
                syn::ReturnType::Type(_, ty) => {
                    let path = get_ty_path(is_result(ty).unwrap());
                    if path.map_or(false, |p| p.segments.last().unwrap().ident == "WaitFuture") {
                        quote! {
                            Ok(wait_from_handle(tokio::spawn(async move {
                                client.#name(args).await
                            })))
                        }
                    } else {
                        quote! {
                            Ok(block_in_place(client.#name(args))
                                .map_err(|e| arci::Error::Other(e.into()))?
                                .into_inner()
                                .into())
                        }
                    }
                }
                syn::ReturnType::Default => unreachable!(),
            };
            quote! {
                #sig {
                    let mut client = self.client.clone();
                    let args = tonic::Request::new(#args);
                    #call
                }
            }
        }
        _ => quote! {},
    });

    quote! {
        impl arci::#trait_name for #client_name {
            #(#methods)*
        }
    }
}

fn gen_server_impl(trait_name: &Ident, item: &ItemTrait, pb_traits: &[ItemTrait]) -> TokenStream {
    const USE_TRY_INTO: &[&str] = &["SystemTime", "Duration"];

    let server_name = format_ident!("Remote{trait_name}Receiver");
    let server_pb_mod = format_ident!("{}_server", trait_name.to_string().to_snake_case());
    let pb_trait = pb_traits.iter().find(|t| t.ident == *trait_name).unwrap();

    let methods = item.items.iter().map(|method| match method {
        syn::TraitItem::Fn(method) => {
            struct ReplacePath;
            impl VisitMut for ReplacePath {
                fn visit_path_mut(&mut self, path: &mut syn::Path) {
                    if path.segments[0].ident == "super" {
                        path.segments[0].ident = format_ident!("pb");
                    }
                    visit_mut::visit_path_mut(self, path);
                }
            }

            let name = &method.sig.ident;
            let arg_len = method.sig.inputs.len() - 1;
            let args: Vec<_> = method
                .sig
                .inputs
                .iter()
                .filter_map(|arg| match arg {
                    syn::FnArg::Receiver(_) => None,
                    syn::FnArg::Typed(arg) => {
                        let pat = &arg.pat;
                        let mut into = quote! { .into() };
                        if let Some(path) = get_ty_path(&arg.ty) {
                            if USE_TRY_INTO
                                .contains(&&*path.segments.last().unwrap().ident.to_string())
                            {
                                into = quote! { .try_into().unwrap() }
                            }
                        }
                        Some(match arg_len {
                            0 => unreachable!(),
                            1 => {
                                if is_str(&arg.ty) {
                                    quote! { &request }
                                } else if matches!(&*arg.ty, syn::Type::Reference(..)) {
                                    quote! { &request #into }
                                } else {
                                    quote! { request #into }
                                }
                            }
                            _ => {
                                if is_str(&arg.ty) {
                                    quote! { &request.#pat }
                                } else if matches!(&*arg.ty, syn::Type::Reference(..)) {
                                    quote! { &request.#pat.unwrap()#into }
                                } else {
                                    quote! { request.#pat.unwrap()#into }
                                }
                            }
                        })
                    }
                })
                .collect();
            let mut pb_method = pb_trait
                .items
                .iter()
                .find_map(|m| {
                    if let syn::TraitItem::Fn(m) = m {
                        if m.sig.ident == *name {
                            return Some(m.clone());
                        }
                    }
                    None
                })
                .unwrap();
            ReplacePath.visit_signature_mut(&mut pb_method.sig);
            let sig = &pb_method.sig;
            let call = match &method.sig.output {
                syn::ReturnType::Type(_, ty) => {
                    let path = get_ty_path(is_result(ty).unwrap());
                    if path.map_or(false, |p| p.segments.last().unwrap().ident == "WaitFuture") {
                        quote! {
                            let res = arci::#trait_name::#name(&self.inner, #(#args),*)
                                .map_err(|e| tonic::Status::unknown(e.to_string()))?
                                .await
                                .map_err(|e| tonic::Status::unknown(e.to_string()))?
                                .into();
                        }
                    } else {
                        quote! {
                            let res = arci::#trait_name::#name(&self.inner, #(#args),*)
                                .map_err(|e| tonic::Status::unknown(e.to_string()))?
                                .into();
                        }
                    }
                }
                syn::ReturnType::Default => unreachable!(),
            };
            quote! {
                #sig {
                    let request = request.into_inner();
                    #call
                    Ok(tonic::Response::new(res))
                }
            }
        }
        _ => quote! {},
    });

    quote! {
        #[tonic::async_trait]
        impl<T> pb::#server_pb_mod::#trait_name for #server_name<T>
        where
            T: arci::#trait_name + 'static,
        {
            #(#methods)*
        }
    }
}

struct CollectTrait<'a>(&'a mut Vec<ItemTrait>);

impl VisitMut for CollectTrait<'_> {
    fn visit_item_trait_mut(&mut self, i: &mut ItemTrait) {
        self.0.push(i.clone());
    }
}
