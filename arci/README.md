# arci

[![crates.io](https://img.shields.io/crates/v/arci.svg?logo=rust)](https://crates.io/crates/arci) [![docs](https://docs.rs/arci/badge.svg)](https://docs.rs/arci) [![docs](https://img.shields.io/badge/docs-main-blue)](https://openrr.github.io/openrr/arci)

Abstract Robot Control Interface.

## How to add trait

1. Add module and trait to `arci/traits`.
2. Add service and message to proto file on `openrr-remote`.
3. Add the new or newly used structure to `openrr-plugin/src/proxy.rs`.
   - If new used structure is `Foo`, you should add `RFoo` structure. 
4. Impl `From<T>` for above struct.
5. Add `From<T>` for `pb::Foo` and `Foo` to `openrr-remote/src/lib.rs`
6. (If need) Add a new structure declaration to the corresponding parts of the two quote! macros in `tools/codegen/src`.
7. Run `./tools/gen-code.sh`

**Codegen will be added.**

## License

Licensed under the [Apache License, Version 2.0](https://github.com/openrr/openrr/blob/main/LICENSE).
