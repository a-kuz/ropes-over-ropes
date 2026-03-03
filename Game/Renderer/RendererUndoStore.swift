import Foundation

final class RendererUndoStore<Entry> {
    private var entries: [Entry] = []

    var canUndo: Bool {
        !entries.isEmpty
    }

    func push(_ entry: Entry) {
        entries.append(entry)
    }

    func pop() -> Entry? {
        entries.popLast()
    }

    func clear() {
        entries.removeAll()
    }
}
