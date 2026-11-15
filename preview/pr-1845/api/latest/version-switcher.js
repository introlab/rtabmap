/* Renders the version dropdown into Doxygen's #projectnumber element.
 *
 * Expects two things set up by the generated HTML header (see Doxyfile.in):
 *   - window.RTABMAP_DOC_ROOT: Doxygen's $relpath^, i.e. the path from the
 *     current page back to the root of *this* version's documentation.
 *   - versions.js loaded from the API root, defining RTABMAP_DOC_VERSIONS.
 *
 * Everything is computed relative to those two, so the same files work for a
 * local preview (file:// or a static server) and for the published site,
 * whatever prefix it is served under.
 */
(function () {
    'use strict';

    function onReady(fn) {
        if (document.readyState !== 'loading') {
            fn();
        } else {
            document.addEventListener('DOMContentLoaded', fn);
        }
    }

    onReady(function () {
        var versions = window.RTABMAP_DOC_VERSIONS;
        var holder = document.getElementById('projectnumber');
        if (!versions || !versions.length || !holder) {
            return; // no version list deployed, or an unexpected Doxygen layout
        }

        // Absolute path of this version's doc root, then of the API root above it.
        var anchor = document.createElement('a');
        anchor.href = window.RTABMAP_DOC_ROOT || './';
        var versionRoot = anchor.pathname.replace(/[^/]*$/, '');  // .../api/<version>/
        var apiRoot = versionRoot.replace(/[^/]+\/$/, '');        // .../api/
        var currentDir = versionRoot.slice(apiRoot.length).replace(/\/$/, '');
        var pageInVersion = window.location.pathname.slice(versionRoot.length);

        var select = document.createElement('select');
        select.className = 'rtabmap-version-switcher';
        select.setAttribute('aria-label', 'Documentation version');

        var matched = false;
        versions.forEach(function (entry) {
            var option = document.createElement('option');
            option.textContent = entry[0];
            option.value = entry[1];
            if (entry[1] === currentDir) {
                option.selected = true;
                matched = true;
            }
            select.appendChild(option);
        });

        // Version not in the list (a local build, or a folder not published yet):
        // show it so the box never lies about which docs are open.
        if (!matched && currentDir) {
            var current = document.createElement('option');
            current.textContent = currentDir;
            current.value = currentDir;
            current.selected = true;
            select.insertBefore(current, select.firstChild);
        }

        select.addEventListener('change', function () {
            // Keep the reader on the same page in the target version. Pages that
            // did not exist back then 404, which is the usual trade-off; the
            // alternative (always landing on index.html) is worse for deep links.
            window.location.href = apiRoot + select.value + '/' + pageInVersion;
        });

        holder.textContent = '';
        holder.appendChild(select);
    });
})();
