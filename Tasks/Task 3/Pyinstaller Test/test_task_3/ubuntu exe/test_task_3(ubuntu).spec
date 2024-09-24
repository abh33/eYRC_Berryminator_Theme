# -*- mode: python ; coding: utf-8 -*-


block_cipher = None


a = Analysis(['test_task_3.py'],
             pathex=['/home/erts/Desktop/new/Harvester_Bot_2-harman/Task 3/evaluation/','/home/erts/miniconda3/envs/py39/lib/python3.9/site-packages/cv2'],
             binaries=[('/home/erts/Desktop/new/Harvester_Bot_2-harman/Task 3/evaluation/sim.py', '.'), ('/home/erts/Desktop/new/Harvester_Bot_2-harman/Task 3/evaluation/simConst.py', '.'), ('/home/erts/Desktop/new/Harvester_Bot_2-harman/Task 3/evaluation/remoteApi.so', '.')],
             datas=[('/home/erts/Desktop/new/Harvester_Bot_2-harman/Task 3/evaluation/task_3_dummy.ttm', '.'), ('/home/erts/Desktop/new/Harvester_Bot_2-harman/Task 3/evaluation/BM_Bot.ttm', '.')],
             hiddenimports=[],
             hookspath=[],
             hooksconfig={},
             runtime_hooks=[],
             excludes=[],
             win_no_prefer_redirects=False,
             win_private_assemblies=False,
             cipher=block_cipher,
             noarchive=False)
pyz = PYZ(a.pure, a.zipped_data,
             cipher=block_cipher)

exe = EXE(pyz,
          a.scripts,
          a.binaries,
          a.zipfiles,
          a.datas,  
          [],
          name='test_task_3',
          debug=False,
          bootloader_ignore_signals=False,
          strip=False,
          upx=True,
          upx_exclude=[],
          runtime_tmpdir=None,
          console=True,
          disable_windowed_traceback=False,
          target_arch=None,
          codesign_identity=None,
          entitlements_file=None )
