use std::{env, path::Path, process::Command};

use anyhow::{bail, Context, Result};

/// Evaluates the given string and returns a concatenated string of the results.
///
/// # Syntax
///
/// Command:
///
/// ```text
/// $(...)
/// ```
///
/// Environment variable:
///
/// ```text
/// ${...}
/// ```
///
/// # Note
///
/// Nesting and escaping are not supported yet.
pub fn evaluate(mut s: &str, current_dir: Option<&Path>) -> Result<String> {
    let mut out = String::new();

    loop {
        match s.find('$') {
            Some(pos) => {
                out.push_str(&s[..pos]);
                s = &s[pos..];
            }
            None => {
                out.push_str(s);
                break;
            }
        }
        match s.as_bytes().get(1) {
            Some(b'(') => {
                let end = match s.find(')') {
                    Some(end) => end,
                    None => bail!("unclosed command literal {:?}", s),
                };
                let script = &s[2..end];
                s = s.get(end + 1..).unwrap_or_default();

                let mut script = script.split(' ');
                let mut cmd = Command::new(script.next().unwrap());
                cmd.args(script);
                if let Some(dir) = current_dir {
                    cmd.current_dir(dir);
                }
                let output = cmd
                    .output()
                    .with_context(|| format!("could not run `{:?}`", cmd))?;
                if !output.status.success() {
                    bail!("`{:?}` didn't exit successfully", cmd);
                }
                let mut output = String::from_utf8(output.stdout)?;
                while output.ends_with('\n') || output.ends_with('\r') {
                    output.pop();
                }
                out.push_str(&output);
            }
            Some(b'{') => {
                let end = match s.find('}') {
                    Some(end) => end,
                    None => bail!("unclosed environment variable literal {:?}", s),
                };
                let key = &s[2..end];
                s = s.get(end + 1..).unwrap_or_default();

                let var = env::var(key)
                    .with_context(|| format!("could not get environment variable {:?}", key))?;
                out.push_str(&var);
            }
            Some(_) => {
                out.push('$');
                s = &s[1..];
            }
            None => {
                out.push('$');
                break;
            }
        }
    }

    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test() {
        assert_eq!(evaluate("a", None).unwrap(), "a");
        assert_eq!(evaluate("$(echo a)", None).unwrap(), "a");
        evaluate("${OPENRR_CONFIG_TEST_ENV}", None).unwrap_err();
        env::set_var("OPENRR_CONFIG_TEST_ENV", "a");
        assert_eq!(evaluate("${OPENRR_CONFIG_TEST_ENV}", None).unwrap(), "a");
    }
}
