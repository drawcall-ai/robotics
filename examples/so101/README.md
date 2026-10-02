# SO101 chess

An SO101 arm with a wrist camera sets up a chessboard's black pieces on randomized desks, using only what a real SO101 could sense: its camera and joint positions.

```sh
pnpm start
```

The CAD assets and derived model data are Apache-2.0 ([assets/LICENSE](assets/LICENSE)); the chess set is CC0 ([assets/chess/LICENSE.txt](assets/chess/LICENSE.txt)). The controller needs models the example does not ship: the [KanisornPutta/chess-model-yolov8m](https://huggingface.co/KanisornPutta/chess-model-yolov8m) piece detector, whose license is unclear (check it before use), and [MobileSAM](https://github.com/ChaoningZhang/MobileSAM) (Apache-2.0) as exported by [Acly/MobileSAM](https://huggingface.co/Acly/MobileSAM) (MIT), both in `models/`.
