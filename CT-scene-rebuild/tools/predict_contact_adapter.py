"""Inspect one labelled source crop with an experimental CPU model."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image
from ct_scene.learned_contact import LearnedContact
p=argparse.ArgumentParser()
for name in ['model','annotations']:p.add_argument('--'+name,type=Path,required=True)
p.add_argument('--id',required=True);a=p.parse_args()
rows=json.loads(a.annotations.read_text())['records'];found=[r for r in rows if r['id']==a.id]
if len(found)!=1:raise ValueError('Expected exactly one matching sample id')
r=found[0];result=LearnedContact(a.model).predict(np.array(Image.open(r['crop']).convert('RGB')),r)
print(json.dumps(dict(id=r['id'],camera=r['camera'],**result),ensure_ascii=False,indent=2))
