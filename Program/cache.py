
from collections import OrderedDict

class LRUCache:
    def __init__(self, max_entries):
        self._entries = OrderedDict()
        self.max_entries = max_entries
        self.hits = 0
        self.misses = 0
        self.evictions = 0

    def __contains__(self, key):
        return key in self._entries

    def __len__(self):
        return len(self._entries)

    def get(self, key, default=None):
        try:
            value = self._entries[key]
        except KeyError:
            self.misses += 1
            return default
        self._entries.move_to_end(key)
        self.hits += 1
        return value

    def __setitem__(self, key, value):
        if key in self._entries:
            self._entries[key] = value
            self._entries.move_to_end(key)
            return
        self._entries[key] = value
        if len(self._entries) > self.max_entries:
            self._entries.popitem(last=False)
            self.evictions += 1

    def statistics(self):
        return {
            "size": len(self),
            "max_entries": self.max_entries,
            "hits": self.hits,
            "misses": self.misses,
            "evictions": self.evictions,
        }

