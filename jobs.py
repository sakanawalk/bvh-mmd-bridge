"""Publish a completed VMD only after its settings and report are durable."""
from pathlib import Path
import json
import hashlib
from core import convert

def run_conversion(source,model,folder,config):
    folder=Path(folder)
    for name in ('motion.vmd','motion.partial.vmd','report.json','settings.json'):
        if (folder/name).exists():raise FileExistsError('既存の変換結果は上書きしません: '+name)
    inputs=[Path(source),Path(model)]
    before=[hashlib.sha256(p.read_bytes()).digest() for p in inputs]
    partial=folder/'motion.partial.vmd'
    report=convert(source,model,partial,config)
    if before != [hashlib.sha256(p.read_bytes()).digest() for p in inputs]:
        raise ValueError('変換中に入力ファイルが変更されました。編集や同期を止めてから再実行してください。')
    (folder/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    (folder/'settings.json').write_text(json.dumps(config,ensure_ascii=False,indent=2),encoding='utf-8')
    partial.rename(folder/'motion.vmd')
    return report
