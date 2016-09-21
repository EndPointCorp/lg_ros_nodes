# Earth smooth transitions extensions

This extension watch over google maps and sends
the message as soon as globe loaded.

## Extension workflow

Extension workflow based on requests handling.
There are two events in that context.

* maps starts to load earth sattelite images
* maps starts to load clouds images

Earth images starts to load when globe hasn't being show,
and we starts the 1sec timeout,
as we get that maps starts to load earth images.

Clouds starts to load when the globe is loaded
and images for globe are mostly loaded.
So we send ready message as we get that maps starts to load clouds.

## Developer nodes

Code for publishing ready message:
```javascript
window.postMessage({ type: "DIRECTOR_WINDOW_READY" }, "*");
```
