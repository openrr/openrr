use arci::Speaker;
use openrr_client::PrintSpeaker;

#[test]
fn test_print_speaker_new() {
    let _ps = PrintSpeaker::new();
}

#[test]
fn test_print_speaker_default() {
    let _ps = PrintSpeaker::default();
}

#[test]
fn test_print_speaker_speak() {
    let ps = PrintSpeaker {};
    ps.speak("test message");
}
