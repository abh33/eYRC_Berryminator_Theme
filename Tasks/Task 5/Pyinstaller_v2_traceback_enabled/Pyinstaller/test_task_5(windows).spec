# -*- mode: python ; coding: utf-8 -*-


block_cipher = None


a = Analysis(['test_task_5.py'],
             pathex=['C:\\Eyantra\\BM\\Task 5\\Pyinstaller_v2_traceback_enabled\\Pyinstaller', 'C:\\Users\\11har\\miniconda3\\envs\\py39\\Lib\\site-packages\\cv2'],
             binaries=[('C:\\Eyantra\\BM\\Task 5\\Pyinstaller_v2_traceback_enabled\\Pyinstaller\\libiconv.dll', '.'), ('C:\\Eyantra\\BM\\Task 5\\Pyinstaller_v2_traceback_enabled\\Pyinstaller\\libzbar-64.dll', '.'), ('C:\\Eyantra\\BM\\Task 5\\Pyinstaller_v2_traceback_enabled\\Pyinstaller\\sim.py', '.'), ('C:\\Eyantra\\BM\\Task 5\\Pyinstaller_v2_traceback_enabled\\Pyinstaller\\simConst.py', '.'), ('C:\\Eyantra\\BM\\Task 5\\Pyinstaller_v2_traceback_enabled\\Pyinstaller\\remoteApi.dll', '.')],
             datas=[('C:\\Eyantra\\BM\\Task 5\\Pyinstaller_v2_traceback_enabled\\Pyinstaller\\task_5_dummy.ttm', '.'), ('C:\\Eyantra\\BM\\Task 5\\Pyinstaller_v2_traceback_enabled\\Pyinstaller\\BM_Bot_general.ttm', '.'), ('C:\\Eyantra\\BM\\Task 5\\Pyinstaller_v2_traceback_enabled\\Pyinstaller\\wall_type_1.ttm', '.'), ('C:\\Eyantra\\BM\\Task 5\\Pyinstaller_v2_traceback_enabled\\Pyinstaller\\wall_type_2.ttm', '.'), ('C:\\Eyantra\\BM\\Task 5\\Pyinstaller_v2_traceback_enabled\\Pyinstaller\\wall_type_3.ttm', '.')],
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
          name='test_task_5',
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
