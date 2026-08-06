"""Marking media assets that carry per-asset timing.

`delay_seconds`/`duration_seconds` live in a scene's activity_config, which only
DirectorMediaBridge sees. AdhocMedia has fixed slots, so there is no field to
carry them through to the pools -- and the pools only need one bit of it:
whether an asset is timed, so they never keep it alive across a scene change.
That bit rides on the asset id.
"""

TIMED_SUFFIX = '_timed'


def mark_timed_id(media_id):
    """Tag an asset id as carrying a delay and/or a duration."""
    if is_timed_id(media_id):
        return media_id
    return '%s%s' % (media_id, TIMED_SUFFIX)


def is_timed_id(media_id):
    """Whether an asset id was tagged by mark_timed_id()."""
    return str(media_id).endswith(TIMED_SUFFIX)
