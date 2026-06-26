class BoatTargetSelector:
    """Select one stable boat-like radar target before camera scheduling."""

    def __init__(self, config):
        self.config = dict(config or {})
        self.track_hits = {}
        self.last_seen_packet = {}
        self.locked_track_id = None

        self.x_min_cm = self._optional_float("x_min_cm")
        self.x_max_cm = self._optional_float("x_max_cm")
        self.y_min_cm = self._optional_float("y_min_cm")
        self.y_max_cm = self._optional_float("y_max_cm")
        self.max_abs_speed_cm_s = self._optional_float("max_abs_speed_cm_s")
        self.min_pv = self._optional_float("min_pv")
        self.max_match_distance_m = self._optional_float("max_match_distance_m")

        self.min_track_hits = int(self.config.get("min_track_hits", 3))
        self.lock_track = bool(self.config.get("lock_track", True))
        self.lock_stale_packets = int(self.config.get("lock_stale_packets", 8))
        self.allow_unconfirmed = bool(self.config.get("allow_unconfirmed", False))

    def _optional_float(self, key):
        value = self.config.get(key)
        if value is None:
            return None
        return float(value)

    def _in_range(self, value, min_value, max_value):
        if min_value is not None and value < min_value:
            return False
        if max_value is not None and value > max_value:
            return False
        return True

    def _passes_filters(self, target):
        x = float(target["x_cm"])
        y = float(target["y_cm"])
        speed = abs(float(target["speed_cm_s"]))
        pv = float(target["pv"])

        if not self._in_range(x, self.x_min_cm, self.x_max_cm):
            return False
        if not self._in_range(y, self.y_min_cm, self.y_max_cm):
            return False
        if self.max_abs_speed_cm_s is not None and speed > self.max_abs_speed_cm_s:
            return False
        if self.min_pv is not None and pv < self.min_pv:
            return False
        if self.max_match_distance_m is not None:
            match_distance = float(target.get("match_distance_m", 0.0))
            if match_distance > self.max_match_distance_m:
                return False
        return True

    def _score(self, target):
        track_id = target.get("track_id")
        hits = self.track_hits.get(track_id, 1)
        pv = float(target.get("pv", 0))
        speed = abs(float(target.get("speed_cm_s", 0)))
        distance = float(target.get("distance_m", 0))

        # Prefer long-lived stable tracks, then stronger returns. Do not penalize
        # low speed heavily because docking or stopping is part of the experiment.
        return hits * 100.0 + pv * 0.2 - speed * 0.01 - distance * 0.1

    def _cleanup(self, packet_no):
        stale_before = packet_no - self.lock_stale_packets
        for track_id, last_seen in list(self.last_seen_packet.items()):
            if last_seen < stale_before:
                self.last_seen_packet.pop(track_id, None)
                self.track_hits.pop(track_id, None)
                if self.locked_track_id == track_id:
                    self.locked_track_id = None

    def select(self, targets, packet_no):
        for target in targets:
            track_id = target.get("track_id")
            if track_id is None:
                continue
            self.track_hits[track_id] = self.track_hits.get(track_id, 0) + 1
            self.last_seen_packet[track_id] = packet_no

        self._cleanup(packet_no)

        candidates = [target for target in targets if self._passes_filters(target)]
        if not candidates:
            return None

        confirmed = [
            target for target in candidates
            if self.track_hits.get(target.get("track_id"), 0) >= self.min_track_hits
        ]
        selectable = confirmed or (candidates if self.allow_unconfirmed else [])
        if not selectable:
            return None

        if self.lock_track and self.locked_track_id is not None:
            for target in selectable:
                if target.get("track_id") == self.locked_track_id:
                    return self._decorate(target, packet_no)

        selected = max(selectable, key=self._score)
        if self.lock_track:
            self.locked_track_id = selected.get("track_id")
        return self._decorate(selected, packet_no)

    def _decorate(self, target, packet_no):
        result = dict(target)
        track_id = result.get("track_id")
        result["boat_candidate"] = True
        result["track_hits"] = self.track_hits.get(track_id, 0)
        result["boat_score"] = round(self._score(result), 3)
        result["selected_packet_no"] = packet_no
        return result
