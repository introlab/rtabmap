/* Single source of truth for the API documentation version dropdown.
 *
 * This file is deployed ONCE at the root of the API docs (e.g. /api/versions.js),
 * NOT inside each version folder. Every published version loads this same file,
 * so adding an entry here makes the new release appear in the dropdown of all
 * previously published versions at once.
 *
 * To publish a new release, add ONE entry at the top and redeploy this file to
 * the API docs root. Format: ['<label>', '<folder name under the API root>'],
 * newest first. Folder names are resolved relative to the API root, so the same
 * file works for a local preview and for the published site, whatever prefix it
 * is served under.
 */
window.RTABMAP_DOC_VERSIONS = [
    ['latest', 'latest'],
];
