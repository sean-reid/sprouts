# Changelog

## 0.1.0 (2026-11-05)


### Features

* **ai:** play regions independently with nimbers over live forms ([#49](https://github.com/sean-reid/sprouts/issues/49)) ([cfdf6f3](https://github.com/sean-reid/sprouts/commit/cfdf6f301225c26107ad7456bc0be2fa7b269012))
* **ai:** route moves inside the face with a constrained Delaunay triangulation ([#51](https://github.com/sean-reid/sprouts/issues/51)) ([ccebc9d](https://github.com/sean-reid/sprouts/commit/ccebc9d9259050ab306e5933c2d3af5e8a8371a2))
* **ai:** route through the face eroded by its clearances ([#64](https://github.com/sean-reid/sprouts/issues/64)) ([e80ebab](https://github.com/sean-reid/sprouts/commit/e80ebab5531baffc5447fb2c1f6971cadf8b1900))
* auto-place the new node, drag to adjust, one path for mouse and touch ([#32](https://github.com/sean-reid/sprouts/issues/32)) ([d49c837](https://github.com/sean-reid/sprouts/commit/d49c8373914b58e6a79351d5d60b9072f54ea279))
* face model for legality, game over and move enumeration ([#22](https://github.com/sean-reid/sprouts/issues/22)) ([b9126f9](https://github.com/sean-reid/sprouts/commit/b9126f9aef96ef8a54c7a490b5541e8c0a965906))
* legality, game over and the search run on the face model ([#35](https://github.com/sean-reid/sprouts/issues/35)) ([c91155e](https://github.com/sean-reid/sprouts/commit/c91155ea7500d4f936b7f32b2bc5f80f8badbce5))
* on-board feedback, turn indicator, and a first-player toggle ([#26](https://github.com/sean-reid/sprouts/issues/26)) ([4dce2fd](https://github.com/sean-reid/sprouts/commit/4dce2fdf52393c7766d37ea1c6ea9e76854a4b4e))
* one canonical spline per line for validation, placement and rendering ([#28](https://github.com/sean-reid/sprouts/issues/28)) ([ad49986](https://github.com/sean-reid/sprouts/commit/ad499860a9aa2e9754f2966f55da479274cf6d8d))
* replay bench with per-phase timings and a Save Game export ([#24](https://github.com/sean-reid/sprouts/issues/24)) ([acbaab7](https://github.com/sean-reid/sprouts/commit/acbaab7a5e5a02780798b3788e4c6c773d0d7c55))
* **route:** fence the parity search so it holds a few parities ([#100](https://github.com/sean-reid/sprouts/issues/100)) ([63e8d4d](https://github.com/sean-reid/sprouts/commit/63e8d4d10edbd27315f0669ba51fea01bc3b4e9d))
* say how many nodes a chord cuts off in the move list ([#69](https://github.com/sean-reid/sprouts/issues/69)) ([92d9bef](https://github.com/sean-reid/sprouts/commit/92d9bef7258345b06a73d1ef5b232afd884d04d0))
* settle the board after each move so crowded games keep room to draw ([#61](https://github.com/sean-reid/sprouts/issues/61)) ([c006076](https://github.com/sean-reid/sprouts/commit/c006076e8830471da88a3fefcb311385ac5e0820))
* **settle:** spread the board past a stuck pair and straighten kinks ([#103](https://github.com/sean-reid/sprouts/issues/103)) ([b2cb153](https://github.com/sean-reid/sprouts/commit/b2cb15362fe3cff9f8c7c2b36521d633c5597b25))
* **share:** carry every move as its index in the engine's move list ([#94](https://github.com/sean-reid/sprouts/issues/94)) ([a4cec90](https://github.com/sean-reid/sprouts/commit/a4cec9052e4d0a0f26dd891fa47db1e28c241ae7))
* **share:** carry routed moves as topology in the share link ([#60](https://github.com/sean-reid/sprouts/issues/60)) ([4541cda](https://github.com/sean-reid/sprouts/commit/4541cda99aaaf89f608a39e64205015b74d8c4c2))
* time-boxed search and any starting node count from 2 to 24 ([#45](https://github.com/sean-reid/sprouts/issues/45)) ([0b7363a](https://github.com/sean-reid/sprouts/commit/0b7363a0c42ba2a7cabe9a86b4b4a592b79b32d2))
* typed worker protocol with serde structs and request cancellation ([#21](https://github.com/sean-reid/sprouts/issues/21)) ([7d0dcc2](https://github.com/sean-reid/sprouts/commit/7d0dcc2be261c12397e3eaf2806665bf5260b45d))
* **ui:** one toolbar row on phones ([#96](https://github.com/sean-reid/sprouts/issues/96)) ([4fe35a4](https://github.com/sean-reid/sprouts/commit/4fe35a414a0937d803bc0dc4fc665c34624a0e49))
* **web:** highlight a move list row's line and nodes on the board ([#47](https://github.com/sean-reid/sprouts/issues/47)) ([71935da](https://github.com/sean-reid/sprouts/commit/71935daad18f8d9e0eb9ffd1cc5d8d556af3dd78))
* **web:** loupe above the finger near existing lines on touch ([#25](https://github.com/sean-reid/sprouts/issues/25)) ([c64c930](https://github.com/sean-reid/sprouts/commit/c64c930399efc76007b1aeb28e5ce5687133b381))
* **web:** move list with replay, saved games, and share links ([#27](https://github.com/sean-reid/sprouts/issues/27)) ([0f9ccc9](https://github.com/sean-reid/sprouts/commit/0f9ccc979800a4f56047cd91df2d6599ca19596b))
* **web:** pan a zoomed board under a live stroke ([#95](https://github.com/sean-reid/sprouts/issues/95)) ([6d6c8c8](https://github.com/sean-reid/sprouts/commit/6d6c8c8287e290fc16aad5d7f7d33719f8c8a819))
* **web:** play from the keyboard with a focus-trapped dialog and reduced motion ([#59](https://github.com/sean-reid/sprouts/issues/59)) ([f86a5a3](https://github.com/sean-reid/sprouts/commit/f86a5a3d6104242d399089a36fa910986f6c8952))
* **web:** share link carries fitted cubics instead of strokes ([#36](https://github.com/sean-reid/sprouts/issues/36)) ([75b80d9](https://github.com/sean-reid/sprouts/commit/75b80d9870a34fab853efdb8323042230ebee8c9))
* **web:** step the starting node count with minus and plus buttons ([#68](https://github.com/sean-reid/sprouts/issues/68)) ([7743f35](https://github.com/sean-reid/sprouts/commit/7743f356638ef4cafa76a26666249ec80ff42936))
* **web:** view transform with zoom, pan, and pointer input ([#23](https://github.com/sean-reid/sprouts/issues/23)) ([efacd8e](https://github.com/sean-reid/sprouts/commit/efacd8ec1add6c2e2be93b7ad54739ffea64d1a6))


### Bug Fixes

* **ai:** bound the reply when the parity search exhausts candidates ([#73](https://github.com/sean-reid/sprouts/issues/73)) ([75cd4fd](https://github.com/sean-reid/sprouts/commit/75cd4fdbf11046b6c6e0581cf8473f4cbe9d9797))
* **ai:** route a short chord whose ends share a guard, and part pinches onto free ground ([#72](https://github.com/sean-reid/sprouts/issues/72)) ([f072e2a](https://github.com/sean-reid/sprouts/commit/f072e2a5f3aac70dbeb3ae39657799276c1596b6))
* **ai:** sweep the cheap pass without parity searches first ([#75](https://github.com/sean-reid/sprouts/issues/75)) ([7675884](https://github.com/sean-reid/sprouts/commit/76758844ee30e663388503274c7de0fcaee3889c))
* **e2e:** wait for a painted frame before reading canvas pixels ([#44](https://github.com/sean-reid/sprouts/issues/44)) ([b515352](https://github.com/sean-reid/sprouts/commit/b515352be531b8fcf2a3c3f9a5ff69ccb5c25378))
* keep the suggested node a node spacing in from the board edge ([#38](https://github.com/sean-reid/sprouts/issues/38)) ([7c28f26](https://github.com/sean-reid/sprouts/commit/7c28f267716ff3570d4c42103c104914d9974794))
* **route:** draw a bare loop from circle points built without trig ([#108](https://github.com/sean-reid/sprouts/issues/108)) ([7a8a681](https://github.com/sean-reid/sprouts/commit/7a8a6814f82f35a1f1b6865ed268924bb91d9e51))
* **route:** keep a relaxed segment where the stored curve cannot cross ([#107](https://github.com/sean-reid/sprouts/issues/107)) ([ec74d5f](https://github.com/sean-reid/sprouts/commit/ec74d5f3a7cc25ee14cb49474c5022e07b5f843c))
* **route:** let a chord leave a corner as narrow as the margin allows ([#99](https://github.com/sean-reid/sprouts/issues/99)) ([f3d0c30](https://github.com/sean-reid/sprouts/commit/f3d0c301065ac76e3984a3fcbc3539e9494a4d39))
* **route:** let the first point after an end node leave its spokes ([#80](https://github.com/sean-reid/sprouts/issues/80)) ([d74db70](https://github.com/sean-reid/sprouts/commit/d74db70361a58d2d953218ced04e6a7c17c30bb1))
* **route:** pin the same-line chord read-back at zero refusals ([#101](https://github.com/sean-reid/sprouts/issues/101)) ([39df43b](https://github.com/sean-reid/sprouts/commit/39df43b9524894af3692c30cfbdeed638b2bccf6))
* **share:** store a settling line as the flattening of its spline ([#79](https://github.com/sean-reid/sprouts/issues/79)) ([6028c7b](https://github.com/sean-reid/sprouts/commit/6028c7bf63d2cc6c9595f3fd13d858eaf908b63b))
* **ui:** give the game over dialog's buttons 44 px targets ([#92](https://github.com/sean-reid/sprouts/issues/92)) ([60e724e](https://github.com/sean-reid/sprouts/commit/60e724ef5903edf585c799a5d7664e5ba6b7a166))
* **ui:** hide the browser's spinner arrows in the node count field ([#77](https://github.com/sean-reid/sprouts/issues/77)) ([4ebcb73](https://github.com/sean-reid/sprouts/commit/4ebcb7390306d978b0fc7636596a68d5d592e4ac))
* **ui:** hold the first player toggle still when the turn changes ([#97](https://github.com/sean-reid/sprouts/issues/97)) ([639fa6b](https://github.com/sean-reid/sprouts/commit/639fa6b6a6dbef49770bae882a1e19aa7f5dd9de))
* **web:** desktop layout, game-over indicator, and first-player toggle ([#37](https://github.com/sean-reid/sprouts/issues/37)) ([77cbac9](https://github.com/sean-reid/sprouts/commit/77cbac922944c0730bfbf3f9ae5301e2b6de6fd1))
* **web:** give the move list the height the square board leaves ([#93](https://github.com/sean-reid/sprouts/issues/93)) ([de249b7](https://github.com/sean-reid/sprouts/commit/de249b7143c5b7acd066fba2abb1219394c27263))
* **web:** highlight moves in white and mark rows with the mover's swatch ([#48](https://github.com/sean-reid/sprouts/issues/48)) ([27ccbeb](https://github.com/sean-reid/sprouts/commit/27ccbeb031463cfd727d664dd5bc4ed2ca98c5d9))
* **web:** overlay the pending count badge so it takes no row space ([#91](https://github.com/sean-reid/sprouts/issues/91)) ([ab4995d](https://github.com/sean-reid/sprouts/commit/ab4995d305bc8d8b6702c1668997b36c377a6a04))
* **web:** say the AI ran out of time when that is why it drew nothing ([#78](https://github.com/sean-reid/sprouts/issues/78)) ([cf14cbf](https://github.com/sean-reid/sprouts/commit/cf14cbf1ff81a1a76f741a5b4b39ec767f36916c))
* **web:** size the board from the play area so WebKit draws it ([#42](https://github.com/sean-reid/sprouts/issues/42)) ([13aa12e](https://github.com/sean-reid/sprouts/commit/13aa12eb9a03e355597e2115a2159eef8b5ddf5e))
* **web:** slide the loupe beside the finger near the top edge ([#90](https://github.com/sean-reid/sprouts/issues/90)) ([8f6ac05](https://github.com/sean-reid/sprouts/commit/8f6ac05f6a4f55a5b9f9eedade450e107662e0c8))


### Performance

* cut the live-form search key cost ([#53](https://github.com/sean-reid/sprouts/issues/53)) ([5df27b9](https://github.com/sean-reid/sprouts/commit/5df27b9698a6e05c6d67323eea4b881e5c4a40e0))
* **share:** cache the shapes a share settles so a refused game shares without the engine ([#111](https://github.com/sean-reid/sprouts/issues/111)) ([e7b917e](https://github.com/sean-reid/sprouts/commit/e7b917e3d892d59188d39e05fc7e741cbcecc8ea))
* **share:** keep each move's index in the saved record so a share never routes again ([#106](https://github.com/sean-reid/sprouts/issues/106)) ([8b31f3d](https://github.com/sean-reid/sprouts/commit/8b31f3d2ecbe8dc5c1049e029b8ba661941b337d))


### Dependencies

* bump web-sys from 0.3.83 to 0.3.107 ([#18](https://github.com/sean-reid/sprouts/issues/18)) ([2a74485](https://github.com/sean-reid/sprouts/commit/2a74485ef35b7830b9f4024f44afd0a0812ef842))
