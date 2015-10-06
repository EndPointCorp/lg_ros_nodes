import json


class NearbyPanos:
    """
    Child class to extend and overwrite the methods of to select nearby panos
    """
    def __init__(self):
        self.panoid = None
        self.metadata = None

    def handle_metadata_msg(self, metadata):
        pass

    def get_metadata(self):
        return None

    def set_panoid(self, panoid):
        self.panoid = panoid

    def find_closest(self, panoid, pov_z):
        return None


class NearbyStreetviewPanos(NearbyPanos):
    def __init__(self):
        self.panoid = None
        self.metadata = None

    def handle_metadata_msg(self, metadata):
        tmp = None
        try:
            tmp = json.loads(metadata.data)
            if tmp['location']['pano'] == self.panoid or self.panoid is None:
                self.metadata = tmp
        except ValueError:
            pass
        except KeyError:
            pass

    def set_panoid(self, panoid):
        if self.panoid != panoid:
            self.metadata = None
        self.panoid = panoid

    def find_closest(self, panoid, pov_z):
        """
        Returns the pano that is closest to the direction pressed on the
        spacenav (either forwards or backwards) based on the nearby panos
        bearing to the current pano
        """
        if not self.get_metadata():
            return None
        if 'links' not in self.metadata or not isinstance(self.metadata['links'], list):
            return None
        self.panoid = panoid
        my_lat = self.metadata['location']['latLng']['lat']
        my_lng = self.metadata['location']['latLng']['lng']
        # set closest to the farthest possible result
        closest = 90
        closest_pano = None
        for data in self.metadata['links']:
            tmp = self.headingDifference(pov_z, float(data['heading']))
            if tmp <= closest:
                closest = tmp
                closest_pano = data['pano']
        return closest_pano

    def headingDifference(self, source, target):
        """
        Finds the difference between two headings, takes into account that
        the value 359 degrees is closer to 0 degrees than 10 degrees is
        """
        diff = abs(target - source) % 360
        return diff if diff < 180 else diff - (diff - 180)

    def get_metadata(self):
        """
        Only return the metadata if it matches the current panoid
        """
        if not self.panoid:
            return None
        if not self.metadata:
            return None
        if 'location' not in self.metadata or 'pano' not in self.metadata['location']:
            return None
        if self.metadata['location']['pano'] != self.panoid:
            return None
        return self.metadata
