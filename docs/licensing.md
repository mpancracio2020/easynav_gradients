# Licensing review (pre-release)

Target license: Apache-2.0. This document records pending checks, not a legal clearance or a new grant of rights.

## Current snapshot

- Root LICENSE and package.xml declare Apache-2.0.
- gradient_planner.cpp and gradient_planner.hpp carry GPL-3.0-or-later notices and the author's copyright.
- The example node and benchmark scripts do not have individual license headers.
- Current upstream EasyNavigation, NavMap and easynav_plugins repositories identify Apache-2.0 as their license (individual packages may differ).

## Release gate

- [ ] Identify exact upstream revisions and any copied or adapted source files, including starter templates.
- [ ] Confirm that the relevant code is available under Apache-2.0 or another compatible grant.
- [ ] Confirm ownership or permission for all original contributions, including university or collaborator rights where applicable.
- [ ] Preserve required copyright, attribution, modification and NOTICE information.
- [ ] Replace outdated GPL headers only where the rights permit doing so, and add SPDX identifiers to cleared files.
- [ ] Review the public release tree and Git history for remaining contradictory notices and third-party material.

If the GPL headers were only inherited boilerplate and all substantive code is original or authorized under Apache-2.0, document that finding before replacing them. If any GPL-only third-party code remains, obtain permission or replace it with independently authored or compatibly licensed code before releasing under Apache-2.0.

## Upstream references

- https://github.com/EasyNavigation/EasyNavigation
- https://github.com/EasyNavigation/NavMap
- https://github.com/EasyNavigation/easynav_plugins
- https://www.apache.org/licenses/LICENSE-2.0

Keep the repository private until the release gate is resolved.
