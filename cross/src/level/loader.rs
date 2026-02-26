use super::definition::LevelDefinition;

pub fn load_from_json(json: &str) -> Option<LevelDefinition> {
    serde_json::from_str(json).ok()
}

pub fn load_embedded(_level_id: usize) -> Option<LevelDefinition> {
    None
}
