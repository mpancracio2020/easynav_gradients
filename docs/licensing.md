# Licensing review (pre-release)

Target license: Apache-2.0. This document records pending checks, not a legal clearance or a new grant of rights.

## Current snapshot

- Root LICENSE and package.xml declare Apache-2.0.
- The initial planner files carried GPL-3.0-or-later notices. Their headers now declare Apache-2.0 on this review branch, based on the author's confirmation that the A* source used Apache-2.0; exact upstream notices remain to be checked.
- The example node and benchmark scripts do not have individual license headers.
- Current upstream EasyNavigation, NavMap and easynav_plugins repositories identify Apache-2.0 as their license (individual packages may differ).

## Release gate

- [ ] Identify exact upstream revisions and any copied or adapted source files, including starter templates.
- [x] Record the author's confirmation that the original A* planner was Apache-2.0.
- [x] Verify Apache-2.0 and Copyright 2025 Intelligent Robotics Lab in both AStarPlanner source files at the pinned reference revision 703b73ae55bf6a419bf309b26084ca523b063ddf.
- [x] Historical starting revision is unknown; the pinned upstream reference revision and copyright notices are documented.
- [x] Maintainer confirmed rights or permissions to distribute contributions under Apache-2.0 on 2026-11-25.
- [ ] Preserve required copyright, attribution, modification and NOTICE information.
- [x] Prepare Apache-2.0 planner headers on the review branch.
- [x] Maintainer confirmed distribution rights for contributions; root Apache-2.0 license applies to original project files.
- [ ] Review the public release tree and Git history for remaining contradictory notices and third-party material.

The author's confirmation supports the proposed header update; it does not independently establish ownership or exhaustively verify all third-party rights. See the provisional NOTICE in the repository root. If any GPL-only third-party code remains, obtain permission or replace it with independently authored or compatibly licensed code before releasing under Apache-2.0.

## Upstream references

- https://github.com/EasyNavigation/EasyNavigation
- https://github.com/EasyNavigation/NavMap
- https://github.com/EasyNavigation/easynav_plugins
- https://www.apache.org/licenses/LICENSE-2.0

Keep the repository private until the maintainer expressly authorizes publication.
