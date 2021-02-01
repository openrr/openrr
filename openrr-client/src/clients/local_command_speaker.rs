use arci::Speaker;
use log::error;
use std::{io, process::Command, str::FromStr};
use thiserror::Error;

#[derive(Debug, Default)]
pub struct LocalCommandSpeaker {
    locale: Locale,
}

impl LocalCommandSpeaker {
    pub fn new(locale: &str) -> Result<Self, LocalCommandSpeakerError> {
        Ok(Self {
            locale: locale.parse()?,
        })
    }

    #[cfg(target_os = "macos")]
    pub fn locale(&mut self, locale: &str) -> Result<&mut Self, LocalCommandSpeakerError> {
        self.locale = locale.parse()?;
        Ok(self)
    }
}

impl Speaker for LocalCommandSpeaker {
    fn speak(&self, message: &str) {
        if let Err(e) = run_local_command(self.locale, message) {
            error!("{}", e)
        }
    }
}

/// ULocale <https://www.localeplanet.com/icu/>
#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Locale {
    Default,
    ar_SA,
    cs_CZ,
    da_DK,
    de_DE,
    el_GR,
    en_AU,
    en_GB,
    en_IE,
    en_IN,
    en_scotland,
    en_US,
    en_ZA,
    es_AR,
    es_ES,
    es_MX,
    fi_FI,
    fr_CA,
    fr_FR,
    he_IL,
    hi_IN,
    hu_HU,
    id_ID,
    it_IT,
    ja_JP,
    ko_KR,
    nb_NO,
    nl_BE,
    nl_NL,
    pt_BR,
    pt_PT,
    ro_RO,
    ru_RU,
    sk_SK,
    sv_SE,
    th_TH,
    tr_TR,
    zh_CN,
    zh_HK,
    zh_TW,
}

impl Default for Locale {
    fn default() -> Self {
        Self::Default
    }
}

#[derive(Debug, Error)]
#[error("{}", .0)]
pub struct LocalCommandSpeakerError(String);

impl FromStr for Locale {
    type Err = LocalCommandSpeakerError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(match s {
            "ar_SA" => Self::ar_SA,
            "cs_CZ" => Self::cs_CZ,
            "da_DK" => Self::da_DK,
            "de_DE" => Self::de_DE,
            "el_GR" => Self::el_GR,
            "en_AU" => Self::en_AU,
            "en_GB" => Self::en_GB,
            "en_IE" => Self::en_IE,
            "en_IN" => Self::en_IN,
            "en_US" => Self::en_US,
            "en_ZA" => Self::en_ZA,
            "en-scotland" => Self::en_scotland,
            "es_AR" => Self::es_AR,
            "es_ES" => Self::es_ES,
            "es_MX" => Self::es_MX,
            "fi_FI" => Self::fi_FI,
            "fr_CA" => Self::fr_CA,
            "fr_FR" => Self::fr_FR,
            "he_IL" => Self::he_IL,
            "hi_IN" => Self::hi_IN,
            "hu_HU" => Self::hu_HU,
            "id_ID" => Self::id_ID,
            "it_IT" => Self::it_IT,
            "ja_JP" => Self::ja_JP,
            "ko_KR" => Self::ko_KR,
            "nb_NO" => Self::nb_NO,
            "nl_BE" => Self::nl_BE,
            "nl_NL" => Self::nl_NL,
            "pt_BR" => Self::pt_BR,
            "pt_PT" => Self::pt_PT,
            "ro_RO" => Self::ro_RO,
            "ru_RU" => Self::ru_RU,
            "sk_SK" => Self::sk_SK,
            "sv_SE" => Self::sv_SE,
            "th_TH" => Self::th_TH,
            "tr_TR" => Self::tr_TR,
            "zh_CN" => Self::zh_CN,
            "zh_HK" => Self::zh_HK,
            "zh_TW" => Self::zh_TW,

            _ => {
                return Err(LocalCommandSpeakerError(format!(
                    "failed to parse locale: {}",
                    s
                )))
            }
        })
    }
}

impl Locale {
    #[cfg(not(target_os = "macos"))]
    fn voice(&self) -> &'static [&'static str] {
        &[]
    }

    #[cfg(target_os = "macos")]
    fn voice(&self) -> &'static [&'static str] {
        // say -v '?'
        // https://gist.github.com/mculp/4b95752e25c456d425c6
        match self {
            Self::Default => &[],
            Self::en_AU => &["Karen"],
            Self::en_GB => &["Daniel"],
            Self::en_US => &["Alex", "Fred", "Samantha", "Victoria"],
            Self::en_IN => &["Rishi", "Veena"],
            Self::en_IE => &["Moira"],
            Self::en_scotland => &["Fiona"],
            Self::en_ZA => &["Tessa"],
            Self::fr_CA => &["Amelie"],
            Self::fr_FR => &["Thomas"],
            Self::nl_NL => &["Xander"],
            Self::nl_BE => &["Ellen"],
            Self::pt_BR => &["Luciana"],
            Self::pt_PT => &["Joana"],
            Self::es_AR => &["Diego"],
            Self::fi_FI => &["Satu"],
            Self::es_MX => &["Paulina", "Juan"],
            Self::es_ES => &["Jorge", "Monica"],
            Self::ar_SA => &["Maged"],
            Self::de_DE => &["Anna"],
            Self::he_IL => &["Carmit"],
            Self::hi_IN => &["Lekha"],
            Self::tr_TR => &["Yelda"],
            Self::hu_HU => &["Mariska"],
            Self::id_ID => &["Damayanti"],
            Self::it_IT => &["Alice", "Luca"],
            Self::ja_JP => &["Kyoko"],
            Self::ro_RO => &["Ioana"],
            Self::sk_SK => &["Laura"],
            Self::sv_SE => &["Alva"],
            Self::th_TH => &["Kanya"],
            Self::zh_TW => &["Mei-Jia"],
            Self::zh_HK => &["Sin-ji"],
            Self::zh_CN => &["Ting-Ting"],
            Self::el_GR => &["Melina"],
            Self::nb_NO => &["Nora"],
            Self::ru_RU => &["Milena", "Yuri"],
            Self::da_DK => &["Sara"],
            Self::cs_CZ => &["Zuzana"],
            Self::ko_KR => &["Yuna"],
        }
    }
}

fn run_local_command(locale: Locale, message: &str) -> io::Result<()> {
    #[cfg(not(target_os = "macos"))]
    let cmd_name = "espeak";
    #[cfg(target_os = "macos")]
    let cmd_name = "say";

    let mut cmd = Command::new(cmd_name);
    if let Some(voice) = locale.voice().first() {
        cmd.args(&["-v", voice]);
    }
    let status = cmd.arg(message).status()?;

    if status.success() {
        Ok(())
    } else if let Some(voice) = locale.voice().get(1) {
        Err(io::Error::new(
            io::ErrorKind::Other,
            format!("faild to run `{}` command with voice `{}`", cmd_name, voice),
        ))
    } else {
        Err(io::Error::new(
            io::ErrorKind::Other,
            format!("faild to run `{}` command with default voice", cmd_name),
        ))
    }
}
