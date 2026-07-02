/*
 * MV3 service worker for the Onboard extension.
 *
 * roslib.js needs a DOM (it pulls in xmlshim, which touches DOMParser /
 * document at load), and a service worker has no DOM. So the rosbridge
 * connection and all publish/subscribe logic live in an offscreen document
 * (see offscreen.html / onboard.js). This worker only keeps that document
 * alive and relays the toolbar-icon toggle to it.
 */

const OFFSCREEN_DOCUMENT_PATH = 'offscreen.html';

async function ensureOffscreenDocument() {
  if (await chrome.offscreen.hasDocument()) {
    return;
  }
  try {
    await chrome.offscreen.createDocument({
      url: OFFSCREEN_DOCUMENT_PATH,
      reasons: ['DOM_PARSER'],
      justification:
        'roslib.js requires a DOM (DOMParser) and maintains the persistent ' +
        'rosbridge WebSocket used to toggle the onboard keyboard.'
    });
  } catch (err) {
    // createDocument throws if a document already exists (e.g. a race between
    // the startup hooks below). That is harmless - the document is up.
    console.error('onboard: could not create offscreen document', err);
  }
}

chrome.runtime.onInstalled.addListener(ensureOffscreenDocument);
chrome.runtime.onStartup.addListener(ensureOffscreenDocument);
// Also create it whenever the worker spins up.
ensureOffscreenDocument();

// In kiosk mode there is no toolbar, but preserve the click-to-toggle behavior.
chrome.action.onClicked.addListener(async function() {
  await ensureOffscreenDocument();
  chrome.runtime.sendMessage({onboard: 'toggle'});
});
