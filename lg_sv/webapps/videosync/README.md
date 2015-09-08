VideoSync
=========

This webapp shows framesync-enabled video.

###Usage

Query arguments are passed to the index to configure the player.

The video source can be any url.

`http://master/videosync/?url=http://foo.com/video.webm`

One instance of the viewsync group must be designated the leader.  All other
instances are followers.  Only the leader plays audio.

`http://master/videosync/?url=http://foo.com/video.webm&leader=true`

All players can toggle pause/play by clicking or touching the video.

###Backend

VideoSync uses [Popcorn.js](http://popcornjs.org/) as the video player.  By
implementing framesync as a Popcorn.js plugin, seek performance is greatly
improved over a simple requestAnimationFrame-based seek.

