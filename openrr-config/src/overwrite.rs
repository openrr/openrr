use std::{iter, mem};

use anyhow::{Context, Result, bail};
use toml::{Value, value};
use toml_query::{delete::TomlValueDeleteExt, insert::TomlValueInsertExt, read::TomlValueReadExt};
use tracing::debug;

const SEPARATORS: &[char] = &['\n', ';'];

/// Replaces the contents of the specified TOML document based on the specified scripts.
///
/// You can specify multiple scripts at once (newline-separated or semicolon-separated).
/// Empty scripts, and leading and trailing separators will be ignored.
///
/// # Set operation
///
/// Syntax:
///
/// ```text
/// <key> = <value>
/// ```
///
/// - If the specified key or array index exists, replace its value.
/// - If the specified key does not exist, create the specified key and value.
/// - If the specified array index does not exist, append the specified value to the array.
/// - If the intermediate data structures do not exist, create them.
///
/// # Delete operation
///
/// Syntax:
///
/// ```text
/// <key> =
/// ```
///
/// - Deletes the specified key and its value or specified array element.
/// - If the specified key or array index does not exist, it will be ignored.
pub fn overwrite(doc: &mut Value, scripts: &str) -> Result<()> {
    let scripts = parse_scripts(scripts)?;

    for script in scripts {
        let query = &script.query;
        let old = doc.read_mut(query)?;
        let exists = old.is_some();
        let is_structure = matches!(&old, Some(r) if r.is_table() || r.is_array());
        match script.operation {
            Operation::Set(value) => {
                // TODO:
                // - Workaround for toml-query bug: https://docs.rs/toml-query/0.10/toml_query/insert/trait.TomlValueInsertExt.html#known-bugs
                // - Validate that the query points to a valid configuration.
                debug!(?query, ?value, "executing insert operation");
                doc.insert(query, value)?;
            }
            Operation::Delete => {
                if !exists {
                    debug!(
                        ?query,
                        "delete operation was not executed because value did not exist"
                    );
                    continue;
                }

                let old = old.unwrap();
                // toml-query does not support deleting non-empty table/array.
                // https://docs.rs/toml-query/0.10/toml_query/delete/trait.TomlValueDeleteExt.html#semantics
                if is_structure {
                    if old.is_array() {
                        *old = Value::Array(vec![]);
                    } else {
                        *old = Value::Table(value::Map::new());
                    }
                }

                debug!(?query, ?is_structure, ?old, "executing delete operation");
                doc.delete(query)?;
            }
        }
    }

    Ok(())
}

/// Replaces the contents of the specified TOML document based on the specified scripts,
/// returning edited document as string.
///
/// See [`overwrite`] for more.
pub fn overwrite_str(doc: &str, scripts: &str) -> Result<String> {
    let mut doc: toml::Value = toml::from_str(doc)?;
    overwrite(&mut doc, scripts)?;
    Ok(toml::to_string(&doc)?)
}

#[derive(Debug)]
struct Script {
    query: String,
    operation: Operation,
}

#[derive(Debug)]
enum Operation {
    Set(Value),
    Delete,
}

fn parse_string_literal(
    buf: &mut String,
    start: char,
    chars: &mut iter::Peekable<impl Iterator<Item = (usize, char)>>,
) -> bool {
    buf.push(start);
    while let Some((_, ch)) = chars.next() {
        buf.push(ch);
        match ch {
            '\\' => {
                if matches!(chars.peek(), Some(&(_, c)) if c == start) {
                    buf.push(chars.next().unwrap().1);
                    continue;
                }
            }
            _ if ch == start => {
                return true;
            }
            _ => {}
        }
    }
    false
}

fn parse_scripts(s: &str) -> Result<Vec<Script>> {
    fn push_script(
        cur_query: &mut Option<String>,
        buf: &mut String,
        scripts: &mut Vec<Script>,
        i: usize,
    ) -> Result<()> {
        let query = cur_query.take().unwrap();
        let value = mem::take(buf);
        let value = value.trim();
        let operation = if value.is_empty() {
            Operation::Delete
        } else {
            let value: Value = toml::from_str(&format!(r#"a = {value}"#))
                .with_context(|| format!("invalid script syntax at {}: {value}", i + 1))?;
            Operation::Set(value["a"].clone())
        };

        scripts.push(Script {
            query: convert_query(&query)?,
            operation,
        });
        Ok(())
    }

    let mut scripts = vec![];

    let mut chars = s.char_indices().peekable();
    let mut cur_query = None;
    let mut in_bracket = 0;
    let mut in_brace = 0;
    let mut buf = String::new();
    while let Some((i, ch)) = chars.next() {
        match ch {
            '"' | '\'' => {
                let end = parse_string_literal(&mut buf, ch, &mut chars);
                if !end {
                    debug!(?buf, ?cur_query, "unexpected eof, expected `{ch}`");
                    bail!("unexpected eof, expected `{ch}`");
                }
            }
            '[' => {
                buf.push(ch);
                in_bracket += 1;
            }
            ']' => {
                buf.push(ch);
                in_bracket -= 1;
            }
            '{' => {
                buf.push(ch);
                in_brace += 1;
            }
            '}' => {
                buf.push(ch);
                in_brace -= 1;
            }
            '=' if in_bracket <= 0 && in_brace <= 0 => {
                if cur_query.is_some() {
                    debug!(?buf, ?i, "expected separator, found `=`");
                    bail!("expected separator, found `=`");
                }
                let query = mem::take(&mut buf);
                cur_query = Some(query);
                debug!(?cur_query);
            }
            _ if in_bracket <= 0 && in_brace <= 0 && SEPARATORS.contains(&ch) => {
                if cur_query.is_none() {
                    if buf.trim().is_empty() {
                        buf.clear();
                        continue;
                    } else {
                        debug!(?buf, ?i, "expected `=`, found separator");
                        bail!("expected `=`, found separator");
                    }
                }
                push_script(&mut cur_query, &mut buf, &mut scripts, i)?;
            }
            _ if ch.is_whitespace() => {}
            _ => {
                buf.push(ch);
            }
        }
    }
    if cur_query.is_none() {
        if !buf.trim().is_empty() {
            debug!(?buf, ?cur_query, ?in_bracket, ?in_brace, "unexpected eof");
            bail!("unexpected eof");
        }
    } else {
        push_script(&mut cur_query, &mut buf, &mut scripts, s.len() - 1)?;
    }

    Ok(scripts)
}

fn convert_query(s: &str) -> Result<String> {
    let mut out = String::with_capacity(s.len());
    let mut chars = s.char_indices().peekable();
    while let Some((_, ch)) = chars.next() {
        match ch {
            '"' | '\'' => {
                let end = parse_string_literal(&mut out, ch, &mut chars);
                assert!(end);
            }
            '[' => {
                if !out.ends_with('.') {
                    out.push('.');
                }
                out.push(ch);
            }
            _ => out.push(ch),
        }
    }

    Ok(out)
}

#[cfg(test)]
mod tests {
    use std::{fs, path::PathBuf};

    use assert_approx_eq::assert_approx_eq;

    use super::*;

    #[test]
    fn test_parse_scripts() {
        let f = |s: &str| parse_scripts(s).unwrap();
        assert!(f("").is_empty());
        assert!(f("\n").is_empty());
        assert!(f(";").is_empty());
        assert!(f(";;").is_empty());

        assert!(parse_scripts("a").is_err());
        assert!(parse_scripts("a\n").is_err());
        assert!(parse_scripts("a;").is_err());
        assert!(parse_scripts(";a").is_err());
        assert!(parse_scripts("a=b").is_err());
        assert!(parse_scripts(r#"a="b"=0"#).is_err());
        assert!(parse_scripts(r#"a=""""#).is_err());

        assert!(matches!(
            &f(r#"a="b""#)[0],
            Script { query, operation: Operation::Set(Value::String(s)), .. }
            if query == "a" && s == "b"
        ));
        assert!(matches!(
            &f(r#"a.b="c=d""#)[0],
            Script { query, operation: Operation::Set(Value::String(s)), .. }
            if query == "a.b" && s == "c=d"
        ));
        assert!(matches!(
            &f(r#"a="""#)[0],
            Script { query, operation: Operation::Set(Value::String(s)), .. }
            if query == "a" && s.is_empty()
        ));
        assert!(matches!(
            &f("a=\"\\\"\"")[0],
            Script { query, operation: Operation::Set(Value::String(s)), .. }
            if query == "a" && s == "\""
        ));
        assert!(matches!(
            &f("a=")[0],
            Script { query, operation: Operation::Delete, .. }
            if query == "a"
        ));

        // array
        assert!(matches!(
            &f(r#"a[0]="""#)[0],
            Script { query, operation: Operation::Set(Value::String(s)), .. }
            if query == "a.[0]" && s.is_empty()
        ));
        assert!(matches!(
            &f(r#"a.[0]="""#)[0],
            Script { query, operation: Operation::Set(Value::String(s)), .. }
            if query == "a.[0]" && s.is_empty()
        ));
        assert!(matches!(
            &f(r#"a[0][1].b[2]="""#)[0],
            Script { query, operation: Operation::Set(Value::String(s)), .. }
            if query == "a.[0].[1].b.[2]" && s.is_empty()
        ));

        // string literal
        assert!(matches!(
            &f(r#"'a'=0"#)[0],
            Script { query, operation: Operation::Set(Value::Integer(n)), .. }
            if query == "'a'" && *n == 0
        ));
        assert!(matches!(
            &f(r#""a"=0"#)[0],
            Script { query, operation: Operation::Set(Value::Integer(n)), .. }
            if query == "\"a\"" && *n == 0
        ));
        assert!(matches!(
            &f(r#"'a=b'=0"#)[0],
            Script { query, operation: Operation::Set(Value::Integer(n)), .. }
            if query == "'a=b'" && *n == 0
        ));
        assert!(matches!(
            &f(r#""a=b"=0"#)[0],
            Script { query, operation: Operation::Set(Value::Integer(n)), .. }
            if query == "\"a=b\"" && *n == 0
        ));

        // separator
        let r = &f("a=0\nb=1");
        assert_eq!(r.len(), 2);
        assert!(matches!(
            &r[0],
            Script { query, operation: Operation::Set(Value::Integer(n)), .. }
            if query == "a" && *n == 0
        ));
        assert!(matches!(
            &r[1],
            Script { query, operation: Operation::Set(Value::Integer(n)), .. }
            if query == "b" && *n == 1
        ));
        let r = &f("a=0;b=1");
        assert_eq!(r.len(), 2);
        assert!(matches!(
            &r[0],
            Script { query, operation: Operation::Set(Value::Integer(n)), .. }
            if query == "a" && *n == 0
        ));
        assert!(matches!(
            &r[1],
            Script { query, operation: Operation::Set(Value::Integer(n)), .. }
            if query == "b" && *n == 1
        ));
        assert!(matches!(
            &f("'a;b'='c;d'")[0],
            Script { query, operation: Operation::Set(Value::String(s)), .. }
            if query == "'a;b'" && s == "c;d"
        ));
        assert!(matches!(
            &f(r#""a;b"="c;d""#)[0],
            Script { query, operation: Operation::Set(Value::String(s)), .. }
            if query == r#""a;b""# && s == "c;d"
        ));
        assert!(matches!(
            &f("a=\"\"\"\nb\nc\n\"\"\"")[0],
            Script { query, operation: Operation::Set(Value::String(s)), .. }
            if query == "a" && s == "b\nc\n"
        ));
    }

    #[test]
    fn test_overwrite() {
        #[track_caller]
        fn read<'a>(v: &'a Value, q: &str) -> Option<&'a Value> {
            v.read(&convert_query(q).unwrap()).unwrap()
        }

        let mut root_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        root_dir.pop(); // openrr-config

        let s = fs::read_to_string(
            root_dir.join("openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml"),
        )
        .unwrap();
        let v: &Value = &toml::from_str(&s).unwrap();

        {
            // set string
            let v = &mut v.clone();
            overwrite(v, r#"urdf_viz_clients_configs[0].name = "a""#).unwrap();
            assert_eq!(
                read(v, "urdf_viz_clients_configs[0].name")
                    .unwrap()
                    .as_str()
                    .unwrap(),
                "a"
            )
        }
        {
            // set string in array
            let v = &mut v.clone();
            overwrite(v, r#"urdf_viz_clients_configs[0].joint_names[0] = "b""#).unwrap();
            assert_eq!(
                read(v, "urdf_viz_clients_configs[0].joint_names[0]")
                    .unwrap()
                    .as_str()
                    .unwrap(),
                "b"
            )
        }
        {
            // set string in table in array
            let v = &mut v.clone();
            overwrite(
                v,
                r#"urdf_viz_clients_configs[0].joint_position_limits[0] = {}"#,
            )
            .unwrap();
            assert!(
                read(v, "urdf_viz_clients_configs[0].joint_position_limits[0]")
                    .unwrap()
                    .as_table()
                    .unwrap()
                    .is_empty(),
            )
        }
        {
            // set string in table in array
            let v = &mut v.clone();
            overwrite(
                v,
                r#"urdf_viz_clients_configs[0].joint_position_limits[1] = { lower = -3.0, upper = 3.0 }"#,
            )
            .unwrap();
            let t = read(v, "urdf_viz_clients_configs[0].joint_position_limits[1]")
                .unwrap()
                .as_table()
                .unwrap();
            assert_approx_eq!(t["lower"].as_float().unwrap(), -3.0);
            assert_approx_eq!(t["upper"].as_float().unwrap(), 3.0);
        }
        {
            // set float
            let v = &mut v.clone();
            overwrite(
                v,
                "urdf_viz_clients_configs[0].joint_position_limits[2].lower = 0.0",
            )
            .unwrap();
            assert_approx_eq!(
                read(
                    v,
                    "urdf_viz_clients_configs[0].joint_position_limits[2].lower"
                )
                .unwrap()
                .as_float()
                .unwrap(),
                0.0
            );
        }
        {
            // delete
            let v = &mut v.clone();
            overwrite(v, "urdf_viz_clients_configs[0].joint_position_limits =").unwrap();
            assert!(read(v, "urdf_viz_clients_configs[0].joint_position_limits").is_none());
        }
        {
            // delete
            let v = &mut v.clone();
            overwrite(
                v,
                "openrr_clients_config.ik_solvers_configs.arm_ik_solver =",
            )
            .unwrap();
            assert!(read(v, "openrr_clients_config.ik_solvers_configs.arm_ik_solver").is_none());
        }
        {
            // delete non exists key
            let v = &mut v.clone();
            overwrite(v, "a.b.c =").unwrap();
        }
        {
            // set array multi-line
            let v = &mut v.clone();
            overwrite(v, "urdf_viz_clients_configs[0].joint_names = [\n\"a\"\n]").unwrap();
            assert_eq!(
                *read(v, "urdf_viz_clients_configs[0].joint_names")
                    .unwrap()
                    .as_array()
                    .unwrap(),
                vec![Value::String("a".into())]
            )
        }
        {
            // set array multi-line
            let v = &mut v.clone();
            overwrite(v, "urdf_viz_clients_configs[0].joint_names = [\n\"a\"]").unwrap();
            assert_eq!(
                *read(v, "urdf_viz_clients_configs[0].joint_names")
                    .unwrap()
                    .as_array()
                    .unwrap(),
                vec![Value::String("a".into())]
            )
        }
        {
            // set array multi-line
            let v = &mut v.clone();
            overwrite(
                v,
                "urdf_viz_clients_configs[0].joint_names = [\n\"a\"]\ndummy=\"\"",
            )
            .unwrap();
            assert_eq!(
                *read(v, "urdf_viz_clients_configs[0].joint_names")
                    .unwrap()
                    .as_array()
                    .unwrap(),
                vec![Value::String("a".into())]
            )
        }
        {
            // TODO: toml-query bug: https://docs.rs/toml-query/0.10/toml_query/insert/trait.TomlValueInsertExt.html#known-bugs
            let v = &mut v.clone();
            overwrite(v, "a[0].b = 0").unwrap_err();
            // assert_eq!(
            //     *read(v, "a[0].b")
            //         .as_integer()
            //         .unwrap(),
            //     vec![Value::Integer(0)]
            // )
        }
        {
            // insert
            let v = &mut v.clone();
            overwrite(
                v,
                "urdf_viz_clients_configs[0].wrap_with_joint_position_limiter =",
            )
            .unwrap();
            assert!(
                v.read("urdf_viz_clients_configs[0].wrap_with_joint_position_limiter")
                    .unwrap()
                    .is_none()
            );
            overwrite(
                v,
                "urdf_viz_clients_configs[0].wrap_with_joint_position_limiter = false",
            )
            .unwrap();
            assert!(
                !read(
                    v,
                    "urdf_viz_clients_configs[0].wrap_with_joint_position_limiter"
                )
                .unwrap()
                .as_bool()
                .unwrap()
            );
        }

        let s = r#"
[gil_gamepad_config.map]
axis_map = [
    ["DPadX", "DPadX"],
    ["LeftStickX", "LeftStickX"],
    ["RightStickX", "RightStickX"],
    ["RightStickY", "RightStickY"],
    ["DPadY", "DPadY"],
    ["LeftStickY", "LeftStickY"],
]
        "#;
        let v: &Value = &toml::from_str(s).unwrap();
        {
            let v = &mut v.clone();
            overwrite(v, r#"gil_gamepad_config.map.axis_map[0][0] = "DPadN""#).unwrap();
            assert_eq!(
                read(v, "gil_gamepad_config.map.axis_map[0][0]")
                    .unwrap()
                    .as_str()
                    .unwrap(),
                "DPadN"
            );
            assert_eq!(
                read(v, "gil_gamepad_config.map.axis_map[0][1]")
                    .unwrap()
                    .as_str()
                    .unwrap(),
                "DPadX"
            );
        }
        {
            let v = &mut v.clone();
            overwrite(
                v,
                "gil_gamepad_config.map.axis_map = [\n[\n\"DPadN\"\n]\n]\n",
            )
            .unwrap();
            assert_eq!(
                read(v, "gil_gamepad_config.map.axis_map[0][0]")
                    .unwrap()
                    .as_str()
                    .unwrap(),
                "DPadN"
            );
            assert!(
                read(v, "gil_gamepad_config.map.axis_map[0]")
                    .unwrap()
                    .as_array()
                    .unwrap()
                    .get(1)
                    .is_none()
            );
        }
    }
}
