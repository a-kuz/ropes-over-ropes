import Foundation
import os.log

enum LevelLoader {
    private static let logger = Logger(subsystem: "com.uzls.four", category: "LevelLoader")
    
    static func load(levelId: Int) -> LevelDefinition? {
        logger.info("Attempting to load level \(levelId)")
        let name = String(format: "level_%03d", levelId)
        logger.info("Looking for file: \(name).json in 'levels' subdirectory")
        
        if let bundlePath = Bundle.main.resourcePath {
            logger.info("Bundle resource path: \(bundlePath)")
            let levelsPath = (bundlePath as NSString).appendingPathComponent("levels")
            logger.info("Expected levels directory: \(levelsPath)")
            let fileManager = FileManager.default
            if fileManager.fileExists(atPath: levelsPath) {
                logger.info("Levels directory exists")
                if let contents = try? fileManager.contentsOfDirectory(atPath: levelsPath) {
                    logger.info("Files in levels directory: \(contents.joined(separator: ", "))")
                }
            } else {
                logger.warning("Levels directory does not exist at: \(levelsPath)")
            }
        }
        
        var url = Bundle.main.url(forResource: name, withExtension: "json", subdirectory: "levels")
        
        if url == nil {
            logger.warning("Level file not found in 'levels' subdirectory, trying root bundle")
            url = Bundle.main.url(forResource: name, withExtension: "json")
        }
        
        guard let url = url else {
            logger.error("Level file not found: \(name).json")
            logger.error("Bundle.main.resourcePath: \(Bundle.main.resourcePath ?? "nil")")
            if let resourceURL = Bundle.main.resourceURL {
                logger.error("Bundle.main.resourceURL: \(resourceURL.path)")
            }
            return nil
        }
        logger.info("Found level file at URL: \(url.path)")
        logger.info("URL isFileURL: \(url.isFileURL)")
        
        guard let data = try? Data(contentsOf: url) else {
            logger.error("Failed to load data from: \(url.path)")
            if let error = try? FileManager.default.attributesOfItem(atPath: url.path) {
                logger.error("File attributes: \(error)")
            } else {
                logger.error("Cannot read file attributes - file may not exist")
            }
            return nil
        }
        logger.info("Loaded \(data.count) bytes from level file")
        if let jsonString = String(data: data, encoding: .utf8) {
            logger.info("JSON content preview (first 200 chars): \(String(jsonString.prefix(200)))")
        }
        
        do {
            let level = try JSONDecoder().decode(LevelDefinition.self, from: data)
            logger.info("Level \(levelId) loaded successfully: \(level.ropes.count) ropes, \(level.holes.count) holes")
            logger.info("Level ID: \(level.id), holeRadius: \(level.holeRadius), particlesPerRope: \(level.particlesPerRope)")
            return level
        } catch {
            logger.error("Failed to decode level \(levelId): \(error.localizedDescription)")
            logger.error("Error type: \(type(of: error))")
            if let decodingError = error as? DecodingError {
                switch decodingError {
                case .keyNotFound(let key, let context):
                    logger.error("Key '\(key.stringValue)' not found at path: \(context.codingPath.map { $0.stringValue }.joined(separator: "."))")
                case .typeMismatch(let type, let context):
                    logger.error("Type mismatch for type \(type) at path: \(context.codingPath.map { $0.stringValue }.joined(separator: "."))")
                case .valueNotFound(let type, let context):
                    logger.error("Value not found for type \(type) at path: \(context.codingPath.map { $0.stringValue }.joined(separator: "."))")
                case .dataCorrupted(let context):
                    logger.error("Data corrupted at path: \(context.codingPath.map { $0.stringValue }.joined(separator: ".")), \(context.debugDescription)")
                @unknown default:
                    logger.error("Unknown decoding error: \(decodingError)")
                }
            }
            return nil
        }
    }
}

