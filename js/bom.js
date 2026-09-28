/*
 * Bill-of-materials renderer for actuator pages.
 *
 * The `bom.json` snapshot that ships with the site is the source of truth
 * here; the page points at it with a data attribute on the mount element:
 *
 *   <p id="bomStatus" class="bom-status">loading…</p>
 *   <div id="bomMount" data-fallback="data/hdp30-bom.json"></div>
 *   <script src="js/bom.js"></script>
 */
const statusEl = document.getElementById('bomStatus');
const mountEl = document.getElementById('bomMount');

const FALLBACK = mountEl.dataset.fallback;

init();

async function init() {
  let bom = null;
  try {
    const res = await fetch(FALLBACK, { cache: 'no-cache' });
    if (res.ok) bom = await res.json();
  } catch (e) { /* fall through to error state */ }

  if (!bom) {
    statusEl.textContent = 'bill of materials failed to load.';
    return;
  }
  statusEl.textContent = '';
  render(bom);
}

function render(bom) {
  const rows = bom.items.map(function (it) {
    const priced = it.packCost != null && it.packQty;
    const per = priced ? (it.packCost / it.packQty).toFixed(2) : null;
    const link = it.link
      ? '<a href="' + it.link + '" target="_blank" rel="noopener">' + it.item + '</a>'
      : '<span class="bom-nolink">' + it.item + '</span>';
    return '<tr class="bom-item"><td>' + link +
      (it.spec ? '<span class="bom-spec">' + it.spec + '</span>' : '') +
      '</td><td class="num">' + it.qty + '</td>' +
      '<td class="num">' + (priced ? '$' + per : '—') + '</td>' +
      '<td class="num">' + (priced ? '$' + (per * it.qty).toFixed(2) : '—') + '</td></tr>';
  }).join('');

  const total = bom.items.reduce(function (s, it) {
    return (it.packCost != null && it.packQty)
      ? s + (it.packCost / it.packQty) * it.qty
      : s;
  }, 0);

  mountEl.innerHTML =
    '<div class="bom-wrap"><table class="spec-table bom-table">' +
    '<thead><tr><th>Item</th><th>Qty</th><th>Each</th><th>Subtotal</th></tr></thead>' +
    '<tbody>' + rows + '</tbody>' +
    '<tfoot><tr><td class="bom-foot-label" colspan="3">hardware per unit</td>' +
    '<td class="num strong">$' + total.toFixed(2) + '</td></tr></tfoot>' +
    '</table></div>' +
    (bom.affiliateDisclosure
      ? '<p class="disclosure">' + bom.affiliateDisclosure + '</p>' : '');
}
