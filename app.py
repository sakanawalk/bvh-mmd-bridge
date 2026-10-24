from pathlib import Path
import json, os, queue, threading, traceback, sys
from datetime import datetime
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from core import BVH, PMX, ROLES, auto_mapping, convert
from jobs import run_conversion


class App:
    def __init__(self, root):
        self.root=root; root.title('BVH → MMD Bridge  |  0.3.0-rc1 公開実験版 r2'); root.geometry('1000x860'); root.minsize(850,720)
        self.events=queue.Queue(); self.busy=False; self.loaded=None; self.last_output=None
        self.bvh=tk.StringVar(); self.pmx=tk.StringVar(); self.output=tk.StringVar(value=str(Path.home()/'Documents'/'BVHBridge_Output'))
        self.scale=tk.StringVar(value='0'); self.height=tk.StringVar(value='0'); self.center=tk.StringVar(value='センター')
        self.axis=tk.StringVar(value='Y-up / Z反転（Motive系）')
        self.contact=tk.BooleanVar(value=True);self.strength=tk.StringVar(value='100');self.floor=tk.StringVar(value='0')
        self.status=tk.StringVar(value='BVHとPMXを選んで「ボーンを確認」を押してください')
        self.controls=[]; self.rows={}
        outer=ttk.Frame(root,padding=18); outer.pack(fill='both',expand=True)
        ttk.Label(outer,text='BVH → MMD Bridge',font=('Yu Gothic UI',23,'bold')).pack(anchor='w')
        ttk.Label(outer,text='BVH＋PMXから モデルの体格に合わせたVMDを作成',font=('Yu Gothic UI',11)).pack(anchor='w',pady=(0,14))
        paths=ttk.Frame(outer); paths.pack(fill='x'); paths.columnconfigure(1,weight=1)
        for i,(label,var,kind) in enumerate([('モーション BVH',self.bvh,'bvh'),('モデル PMX',self.pmx,'pmx'),('出力フォルダー',self.output,'folder')]):
            ttk.Label(paths,text=label,width=19).grid(row=i,column=0,sticky='w',pady=4)
            entry=ttk.Entry(paths,textvariable=var); entry.grid(row=i,column=1,sticky='ew',padx=8); self.controls.append(entry)
            btn=ttk.Button(paths,text='選択',command=lambda v=var,k=kind:self.choose(v,k)); btn.grid(row=i,column=2); self.controls.append(btn)
        bar=ttk.Frame(outer); bar.pack(fill='x',pady=10)
        for label,command in [('ボーンを確認',self.inspect),('設定を読み込む',self.load_profile),('設定を保存',self.save_profile)]:
            btn=ttk.Button(bar,text=label,command=command); btn.pack(side='left',padx=(0,8)); self.controls.append(btn)
        settings=ttk.LabelFrame(outer,text='変換設定',padding=10); settings.pack(fill='x')
        for col,(label,var,width) in enumerate([('移動ボーン',self.center,15),('倍率 0＝脚長から自動',self.scale,10),('高さ補正（MMD単位）',self.height,10)]):
            ttk.Label(settings,text=label).grid(row=0,column=col*2,padx=4)
            ent=ttk.Entry(settings,textvariable=var,width=width); ent.grid(row=0,column=col*2+1,padx=4); self.controls.append(ent)
        ttk.Label(settings,text='座標系').grid(row=1,column=0,pady=8)
        axes=ttk.Combobox(settings,textvariable=self.axis,state='readonly',width=30,values=['Y-up / Z反転（Motive系）','Y-up / 変換なし','Z-up / YとZを交換']); axes.grid(row=1,column=1,columnspan=3,sticky='w'); self.controls.append(axes)
        check=ttk.Checkbutton(settings,text='足の接地補正',variable=self.contact);check.grid(row=2,column=0,sticky='w');self.controls.append(check)
        ttk.Label(settings,text='強さ（0〜100%）').grid(row=2,column=1)
        strength=ttk.Entry(settings,textvariable=self.strength,width=8);strength.grid(row=2,column=2);self.controls.append(strength)
        ttk.Label(settings,text='床高さ（MMD単位）').grid(row=2,column=3)
        floor=ttk.Entry(settings,textvariable=self.floor,width=8);floor.grid(row=2,column=4);self.controls.append(floor)
        box=ttk.LabelFrame(outer,text='ボーン対応  空欄の役割は出力しません',padding=6); box.pack(fill='both',expand=True,pady=12)
        canvas=tk.Canvas(box,highlightthickness=0); scrollbar=ttk.Scrollbar(box,orient='vertical',command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set); scrollbar.pack(side='right',fill='y'); canvas.pack(side='left',fill='both',expand=True)
        self.table=ttk.Frame(canvas); window=canvas.create_window((0,0),window=self.table,anchor='nw')
        self.table.bind('<Configure>',lambda e:canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>',lambda e:canvas.itemconfigure(window,width=e.width))
        for i,title in enumerate(['役割','BVHボーン','PMXボーン']): ttk.Label(self.table,text=title,font=('Yu Gothic UI',10,'bold')).grid(row=0,column=i,sticky='w',padx=8,pady=4)
        self.table.columnconfigure(1,weight=1); self.table.columnconfigure(2,weight=1)
        for i,role in enumerate(ROLES,1):
            ttk.Label(self.table,text=role,width=20).grid(row=i,column=0,sticky='w',padx=8)
            a=ttk.Combobox(self.table,state='readonly'); z=ttk.Combobox(self.table,state='readonly')
            a.grid(row=i,column=1,sticky='ew',padx=5,pady=2); z.grid(row=i,column=2,sticky='ew',padx=5,pady=2)
            self.rows[role]=(a,z); self.controls.extend((a,z))
        ttk.Label(outer,text='接地中の足を保持し 膝・腰の補正をFKへ焼き込みます（MMDの足IKはオフ）\n平らな床向けです  PMXの靴底形状を考慮します  指・表情・物理は生成しません',foreground='#765319').pack(anchor='w')
        footer=ttk.Frame(outer); footer.pack(fill='x',pady=(12,0))
        self.run=ttk.Button(footer,text='VMDを書き出す',command=self.start); self.run.pack(side='left'); self.controls.append(self.run)
        ttk.Button(footer,text='出力を開く',command=self.open_output).pack(side='left',padx=10)
        ttk.Label(outer,textvariable=self.status,wraplength=900).pack(anchor='w',pady=(10,0))
        root.protocol('WM_DELETE_WINDOW',self.close); root.after(150,self.poll)

    def choose(self,var,kind):
        path=filedialog.askdirectory() if kind=='folder' else filedialog.askopenfilename(filetypes=[(kind.upper(),'*.'+kind)])
        if path: var.set(path)

    def inspect(self):
        try:
            b=BVH(self.bvh.get()); m=PMX(self.pmx.get()); mapping=auto_mapping(b,m)
            for role,(a,z) in self.rows.items():
                a.configure(values=['']+list(b.names)); z.configure(values=['']+list(m.names))
                a.set(mapping.get(role,{}).get('source','')); z.set(mapping.get(role,{}).get('target',''))
            self.loaded=(self.bvh.get(),self.pmx.get())
            self.status.set(f'BVH {b.count:,}フレーム / {(b.count-1)*b.dt:.3f}秒  PMX {m.name}  自動対応 {len(mapping)}関節')
        except Exception as e: messagebox.showerror('読み込みエラー',str(e)); self.loaded=None

    def config(self):
        if self.loaded!=(self.bvh.get(),self.pmx.get()): raise ValueError('ファイル変更後は「ボーンを確認」を押してください')
        mapping={}
        for role,(a,z) in self.rows.items():
            if bool(a.get()) != bool(z.get()): raise ValueError(role+' のBVH・PMXを両方選んでください')
            if a.get(): mapping[role]=dict(source=a.get(),target=z.get())
        bases=[[[1,0,0],[0,1,0],[0,0,-1]],[[1,0,0],[0,1,0],[0,0,1]],[[1,0,0],[0,0,1],[0,1,0]]]
        labels=['Y-up / Z反転（Motive系）','Y-up / 変換なし','Z-up / YとZを交換']
        return dict(mapping=mapping,center=self.center.get(),scale=float(self.scale.get()),height_offset=float(self.height.get()),basis=bases[labels.index(self.axis.get())],axis_label=self.axis.get(),contact_enabled=self.contact.get(),contact_strength=float(self.strength.get())/100,floor_height=float(self.floor.get()))

    def save_profile(self):
        try:
            config=self.config(); path=filedialog.asksaveasfilename(defaultextension='.json',filetypes=[('変換設定','*.json')])
            if path: Path(path).write_text(json.dumps(config,ensure_ascii=False,indent=2),encoding='utf-8'); self.status.set('変換設定を保存しました')
        except Exception as e: messagebox.showerror('設定保存エラー',str(e))

    def load_profile(self):
        if self.loaded!=(self.bvh.get(),self.pmx.get()): messagebox.showinfo('先にファイルを確認','BVHとPMXを選び「ボーンを確認」を押してください'); return
        path=filedialog.askopenfilename(filetypes=[('変換設定','*.json')])
        if not path:return
        try:
            cfg=json.loads(Path(path).read_text(encoding='utf-8'))
            for role,(a,z) in self.rows.items():
                v=cfg['mapping'].get(role,{})
                for widget,key in [(a,'source'),(z,'target')]:
                    value=v.get(key,'')
                    if value not in widget['values']: raise ValueError('現在のファイルに存在しないボーン: '+value)
            for role,(a,z) in self.rows.items():
                v=cfg['mapping'].get(role,{}); a.set(v.get('source','')); z.set(v.get('target',''))
            self.scale.set(str(cfg.get('scale',0))); self.height.set(str(cfg.get('height_offset',0))); self.center.set(cfg.get('center','センター')); self.axis.set(cfg.get('axis_label','Y-up / Z反転（Motive系）'))
            self.contact.set(cfg.get('contact_enabled',False));self.strength.set(str(cfg.get('contact_strength',1)*100));self.floor.set(str(cfg.get('floor_height',0)))
            self.status.set('設定を読み込みました')
        except Exception as e: messagebox.showerror('設定読み込みエラー',str(e))

    def start(self):
        if self.busy:return
        try:
            config=self.config(); folder=Path(self.output.get()).expanduser()
            if not self.output.get().strip(): raise ValueError('出力フォルダーを指定してください')
            src=self.bvh.get(); model=self.pmx.get()
            run=folder/(Path(src).stem+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')); run.mkdir(parents=True,exist_ok=False)
        except Exception as e: messagebox.showerror('変換設定エラー',str(e)); return
        self.busy=True; self.status.set('変換中です… 長いモーションは数分かかる場合があります')
        for widget in self.controls: widget.configure(state='disabled')
        def worker():
            try:
                report=run_conversion(src,model,run,config)
                self.events.put(('done',run,report))
            except Exception as e:
                try:(run/'error.txt').write_text(traceback.format_exc(),encoding='utf-8')
                except OSError:pass
                self.events.put(('error',run,str(e)))
        threading.Thread(target=worker,daemon=True).start()

    def poll(self):
        try:
            kind,folder,result=self.events.get_nowait(); self.busy=False; self.last_output=folder
            for widget in self.controls: widget.configure(state='readonly' if isinstance(widget,ttk.Combobox) else 'normal')
            if kind=='done':
                contact=result.get('contact',{});note=''
                if contact.get('enabled'):
                    sides=contact['sides'];count=sum(v['plant_frames'] for v in sides.values())
                    note=f'  接地補正 左{sides["Left"]["plant_frames"]} / 右{sides["Right"]["plant_frames"]}フレーム' if count else '  接地対象なし（入力の動きを維持）'
                    unreachable=sum(v['unreachable_frames'] for v in sides.values())
                    if unreachable:note+=f'  脚長制限 {unreachable}件：詳細はreport.json'
                self.status.set(f'書き出し完了  {result["frames"]:,}フレーム / 30fps{note}\n{folder / "motion.vmd"}')
            else: self.status.set('変換できませんでした  エラーログを出力しました'); messagebox.showerror('変換エラー',result)
        except queue.Empty: pass
        self.root.after(150,self.poll)

    def open_output(self):
        folder=self.last_output or Path(self.output.get())
        if folder.is_dir(): os.startfile(folder)
        else: messagebox.showinfo('出力フォルダー','まだ出力フォルダーがありません')

    def close(self):
        if self.busy: messagebox.showinfo('変換中','書き出しが完了するまで待ってください'); return
        self.root.destroy()


if __name__=='__main__':
    if len(sys.argv)==3 and sys.argv[1]=='--verify-job':
        created=False
        try:
            job=json.loads(Path(sys.argv[2]).read_text(encoding='utf-8-sig'))
            folder=Path(job['output']);folder.mkdir(parents=True,exist_ok=False);created=True
            run_conversion(job['bvh'],job['pmx'],folder,job.get('config',{}))
        except Exception:
            if created:
                try:(folder/'error.txt').write_text(traceback.format_exc(),encoding='utf-8')
                except OSError:pass
            sys.exit(1)
    elif len(sys.argv)==3 and sys.argv[1]=='--smoke-test':
        root=tk.Tk(); root.withdraw(); application=App(root); root.update()
        Path(sys.argv[2]).write_text(json.dumps({'packaged_gui_startup':True,'roles':len(application.rows)}),encoding='utf-8')
        root.destroy()
    else:
        root=tk.Tk(); App(root); root.mainloop()
