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
/// If the environment variable is not set, it will be replaced by an empty string.
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
                    bail!(
                        "`{:?}` didn't exit successfully\nstdout:\n{}\n\nstderr:\n{}\n",
                        cmd,
                        String::from_utf8_lossy(&output.stdout),
                        String::from_utf8_lossy(&output.stderr)
                    );
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

                let var = match env::var(key) {
                    Err(env::VarError::NotPresent) => String::new(),
                    res => res
                        .with_context(|| format!("could not get environment variable {:?}", key))?,
                };
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

/// Evaluates the given string and returns a concatenated string of the results.
/// Unlike [`evaluate`], the text will be evaluated by `bash`.
pub fn evaluate_bash(s: &str, current_dir: Option<&Path>) -> Result<String> {
    if s.contains("$(") {
        let mut cmd = Command::new("bash");
        cmd.args([
            "--noprofile",
            "--norc",
            "-eo",
            "pipefail",
            "-c",
            &format!("echo -n {}", s),
        ]);
        if let Some(dir) = current_dir {
            cmd.current_dir(dir);
        }
        let output = cmd
            .output()
            .with_context(|| format!("could not run `{:?}`", cmd))?;
        if !output.status.success() {
            bail!(
                "`{:?}` didn't exit successfully\nstdout:\n{}\n\nstderr:\n{}\n",
                cmd,
                String::from_utf8_lossy(&output.stdout),
                String::from_utf8_lossy(&output.stderr)
            );
        }
        let output = String::from_utf8(output.stdout)?;
        Ok(output)
    } else {
        evaluate(s, current_dir)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test() {
        for (i, evaluate) in [evaluate, evaluate_bash].into_iter().enumerate() {
            assert_eq!(evaluate("a", None).unwrap(), "a");
            assert_eq!(evaluate("a$", None).unwrap(), "a$");
            assert_eq!(evaluate("$a", None).unwrap(), "$a");
            assert_eq!(evaluate("$(echo a)", None).unwrap(), "a");
            assert_eq!(evaluate("$(echo a)b", None).unwrap(), "ab");
            if i == 0 {
                assert_eq!(evaluate("$(echo a))", None).unwrap(), "a)");
                assert_eq!(evaluate("$$(echo a)", None).unwrap(), "$a");
                // TODO: should be "a  b c  d"
                assert_eq!(
                    evaluate("a  b $(echo \"c  d\")", None).unwrap(),
                    "a  b \"c  d\""
                );
            } else {
                evaluate("$(echo a))", None).unwrap_err();
                evaluate("$$(echo a)", None).unwrap_err();
                // TODO: should be "a  b c  d"
                assert_eq!(evaluate("a  b $(echo \"c  d\")", None).unwrap(), "a b c d");
            }
            assert_eq!(
                evaluate("$(echo a)", Some(&env::current_dir().unwrap())).unwrap(),
                "a"
            );
            evaluate("$(echo a", None).unwrap_err();
            assert_eq!(evaluate("${OPENRR_CONFIG_TEST_ENV}", None).unwrap(), "");
            env::set_var("OPENRR_CONFIG_TEST_ENV", "a");
            assert_eq!(evaluate("${OPENRR_CONFIG_TEST_ENV}", None).unwrap(), "a");
            assert_eq!(evaluate("${OPENRR_CONFIG_TEST_ENV}b", None).unwrap(), "ab");
            assert_eq!(evaluate("${OPENRR_CONFIG_TEST_ENV}}", None).unwrap(), "a}");
            assert_eq!(
                evaluate("$${OPENRR_CONFIG_TEST_ENV}}", None).unwrap(),
                "$a}"
            );
            evaluate("${OPENRR_CONFIG_TEST_ENV", None).unwrap_err();
            env::remove_var("OPENRR_CONFIG_TEST_ENV");
        }
    }

    #[test]
    fn test_evaluate_bash() {
        assert_eq!(evaluate_bash("$(echo $(echo a))", None).unwrap(), "a");
        assert_eq!(
            evaluate_bash(
                "$(if [[ -z ${a:-} ]]; then\necho a\nelse\necho b\nfi)",
                None
            )
            .unwrap(),
            "a"
        );
        assert_eq!(
            evaluate_bash(
                "$(a=1; if [[ -z ${a:-} ]]; then\necho a\nelse\necho b\nfi)",
                None
            )
            .unwrap(),
            "b"
        );
        assert_eq!(
            evaluate_bash("$(case ${a:-} in\n*) echo a\n;;\nesac)", None).unwrap(),
            "a"
        );
    }
}
