"""Map-scoped camera viewing poses, kept beside the saved map."""
import hashlib
import json
import uuid
from pathlib import Path

from .simulation import validate_pose
from .storage import atomic_json


class ObservationPoints:
    def __init__(self, directory):
        self.directory = Path(directory) / 'observation_points'

    def path(self, map_id):
        return self.directory / (hashlib.sha256(map_id.encode()).hexdigest() + '.json')

    @staticmethod
    def validate(points):
        if not isinstance(points, list) or len(points) > 20:
            raise ValueError('Save at most 20 observation points')
        saved, ids = [], set()
        for point in points:
            if not isinstance(point, dict) or set(point) != {'id', 'x_m', 'y_m', 'yaw_rad'}:
                raise ValueError('Observation point requires id, x_m, y_m and yaw_rad')
            if not isinstance(point['id'], str):
                raise ValueError('Observation point ID must be a UUID')
            identity = str(uuid.UUID(point['id']))
            if identity in ids:
                raise ValueError('Duplicate observation point ID')
            ids.add(identity)
            saved.append(dict(id=identity, **validate_pose({k: v for k, v in point.items() if k != 'id'})))
        return saved

    def load(self, map_id):
        path = self.path(map_id)
        if not path.exists():
            return dict(map_id=map_id, revision=0, points=[])
        value = json.loads(path.read_text())
        if not isinstance(value, dict) or value.get('map_id') != map_id or type(value.get('revision')) is not int or value['revision'] < 0:
            raise ValueError('Invalid saved observation points')
        return {**value, 'points': self.validate(value.get('points'))}

    def save(self, map_id, points, revision):
        value = dict(map_id=map_id, revision=revision, points=self.validate(points))
        atomic_json(self.path(map_id), value)
        return value

    def migrate(self, map_id, update):
        if self.path(map_id).exists() or not update or update.get('map_id') != map_id or not update.get('goals'):
            return
        points = [dict(id=str(uuid.uuid5(uuid.NAMESPACE_URL, f'{map_id}:view:{i}')), **pose)
                  for i, pose in enumerate(update['goals'])]
        self.save(map_id, points, 1)
