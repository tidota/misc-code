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
