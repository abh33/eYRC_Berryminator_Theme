# -*- mode: python ; coding: utf-8 -*-


block_cipher = None


a = Analysis(['test_task_4.py'],
             pathex=['/home/eyrc/Desktop/Task 4 exec', '/home/eyrc/BM_1234/lib/python3.9/site-packages/cv2', '/home/eyrc/BM_1234/lib/python3.9/site-packages/gspread'],
             binaries=[('/home/eyrc/Desktop/Task 4 exec/sim.py', '.'),
                       ('/home/eyrc/Desktop/Task 4 exec/simConst.py', '.'), 
                       ('/home/eyrc/Desktop/Task 4 exec/remoteApi.so', '.'),],
             datas=[('/home/eyrc/Desktop/Task 4 exec/task_4_dummy.ttm', '.'),
                    ('/home/eyrc/Desktop/Task 4 exec/BM_Bot_general.ttm', '.')],
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
          name='test_task_4',
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
