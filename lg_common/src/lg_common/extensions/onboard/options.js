function save_options() {
    var defaults = document.getElementById('defaults').value;
    chrome.storage.sync.set({
        defaults: defaults
    }, function() {
        // Update status to let user know options were saved.
        var status = document.getElementById('status');
        status.textContent = 'Options saved.';
        setTimeout(function() {
            status.textContent = '';
        }, 750);
    });
}

function restore_options() {
  chrome.storage.sync.get({
    defaults: 'foobar'
  }, function(items) {
    document.getElementById('defaults').checked = items.defaults;
  });
}
document.addEventListener('DOMContentLoaded', restore_options);
document.getElementById('save').addEventListener('click', save_options);
