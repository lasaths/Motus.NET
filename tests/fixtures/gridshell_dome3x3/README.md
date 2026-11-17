# 3×3 gridshell dome (example data)

`dome3x3.site.json` is a `BambooTrussLayout.Site` (System.Text.Json, property names as in the record).

- Grid: 4 × 4 nodes (P00…P33) on a spherical cap. The plan cell is 0.30 m, turned 45°. Span / rise is 6. The corner nodes are at y = 0.19 m.
- Canes: 24 members. Each cane is node distance − 2 × 0.03 m gap (0.240–0.256 m). `Site.Length` is the longest cane.
- Anchors: the 12 boundary nodes. The clamp is 0.10 m below the node. The robot starts on `anchor-P00`.
- Stock: each member has its own stock spot (`StructureMember.Feed`), except P00P10 and P00P01. Those two use the site stock, because the robot cannot walk before its first build.
  An offline scout selected the spots under the dome. For each member, the spot gives the most anchor and cane footholds that reach its stock and its place by IK, with a clear stock grasp.
- Stock height (`FeedTop`): 0.12 m. Ground: flat.

Use: see "3×3 gridshell dome example" in the repository README.
