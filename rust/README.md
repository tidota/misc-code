# personal notes about Rust

This is my personal notes to get started with Rust.

The tutorial website: https://doc.rust-lang.org/book/ch01-00-getting-started.html

# Installation/Preparation

To install Rust compiler
```
curl --proto '=https' --tlsv1.3 https://sh.rustup.rs -sSf | sh
```

To check the version
```
rustc --version
```

To update Rust
```
rustup update
```

To check the documentation
```
rustup doc
```
Note that some hack is necessary for Windows/WSL
https://azechi-n.hatenadiary.com/entry/2021/01/01/142027

To uninstall Rust
```
rustup self uninstall
```

# Coding

The extension of Rust code file is `.rs`.

To print something as "hello world."
```
fn main() {
    println!("Hello, world!");
}
```
(why on earth is there an exclamation point there??)

# Compilation

```
rustc <source file>
```
For example,
```
rustc main.rs
```

# Cargo

To check the version
```
cargo --version
```

To create a new project
```
cargo new project_name
```

To check (check if it is compilable)
```
cargo check
```
To build and run
```
cargo run
```

To build
```
cargo build
```
To build a release version
```
cargo build --release
```
