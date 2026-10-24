"""Publish a completed VMD only after its settings and report are durable."""
from pathlib import Path
import json
from core import convert

def run_conversion(source,model,folder,config):
    folder=Path(folder)
    for name in ('motion.vmd','motion.partial.vmd','report.json','settings.json'):
        if (folder/name).exists():raise FileExistsError('既存の変換結果は上書きしません: '+name)
    partial=folder/'motion.partial.vmd'
    report=convert(source,model,partial,config)
    (folder/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    (folder/'settings.json').write_text(json.dumps(config,ensure_ascii=False,indent=2),encoding='utf-8')
    partial.rename(folder/'motion.vmd')
    return report
