import Foundation

enum LevelLoader {
    static func load(levelId: Int) -> LevelDefinition? {
        let name = String(format: "level_%03d", levelId)
        guard let url = Bundle.main.url(forResource: name, withExtension: "json", subdirectory: "levels") else {
            return nil
        }
        guard let data = try? Data(contentsOf: url) else { return nil }
        return try? JSONDecoder().decode(LevelDefinition.self, from: data)
    }
}

