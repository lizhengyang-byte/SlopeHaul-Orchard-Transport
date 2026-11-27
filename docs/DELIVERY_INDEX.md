# 交付索引

- GitHub 仓库：https://github.com/lizhengyang-byte/SlopeHaul-Orchard-Transport （public，MIT）
- 飞书文件夹：https://my.feishu.cn/drive/folder/UwWXf0kmHlwbkidTKtHcVIAfnlg
- 参赛人：李尚儒（成都锦城学院，工程管理）

## 本地复现三步

```bash
pip install -r requirements.txt
python scripts/run_benchmark.py   # 五场景 × 基线/本方案，约 40 s，输出 results/
python tests/test_smoke.py        # 7 项冒烟测试
```

权重文件 `models/soil_mlp.json` 缺失时，`run_benchmark.py` 会按固定种子自动训练一份，保证他人复现同一结果。

## 文档对应关系

| 飞书文档 | 仓库文件 |
|---|---|
| 00_项目总览（README） | README.md |
| 01_设计说明 | docs/DESIGN.md |
| 02_结果与解读 | docs/RESULTS.md |
| 03_参考项目与复刻边界 | docs/REFERENCES.md |

## 五场景结论速览（seed=1）

| 场景 | 基线 | 本方案 |
|---|---|---|
| S1 松软土重载爬坡 | 失败，40 s 只到 2.8 m | 到达 24.2 s，滑移近零 |
| S2 横坡弯道 | 到达 24.8 s | 到达 15.3 s |
| S3 18° 松软坡驻车 | 失败，4.9 s 失控 | 到达 24.5 s，零后溜 |
| S4 起伏下坡 | 到达 29.2 s | 到达 32.0 s（限速更保守） |
| S5 26° 湿滑坡 | 被动滑停 3.2 m | 主动拦停 4.1 m，P95 滑移 0 |

平均滑移率 10.85% → 0.02%，到达率 2/5 → 4/5，倾翻 0 次。
