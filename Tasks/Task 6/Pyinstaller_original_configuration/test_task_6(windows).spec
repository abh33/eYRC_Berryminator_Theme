# -*- mode: python ; coding: utf-8 -*-


block_cipher = None


a = Analysis(['test_task_6.py'],
             pathex=['C:\\Users\\Atul Sahay\\Desktop\\BM\\Task 6\\evaluation', 'C:\\Users\\Atul Sahay\\miniconda3\\envs\\py39\\Lib\\site-packages\\cv2'],
             binaries=[('C:\\Users\\Atul Sahay\\Desktop\\BM\\Task 6\\evaluation\\libiconv.dll', '.'), ('C:\\Users\\Atul Sahay\\Desktop\\BM\\Task 6\\evaluation\\libzbar-64.dll', '.'), ('C:\\Users\\Atul Sahay\\Desktop\\BM\\Task 6\\evaluation\\sim.py', '.'), ('C:\\Users\\Atul Sahay\\Desktop\\BM\\Task 6\\evaluation\\simConst.py', '.'), ('C:\\Users\\Atul Sahay\\Desktop\\BM\\Task 6\\evaluation\\remoteApi.dll', '.')],
             datas=[('C:\\Users\\Atul Sahay\\Desktop\\BM\\Task 6\\evaluation\\task_6_dummy.ttm', '.'), ('C:\\Users\\Atul Sahay\\Desktop\\BM\\Task 6\\evaluation\\BM_Bot_general.ttm', '.'), ('C:\\Users\\Atul Sahay\\Desktop\\BM\\Task 6\\evaluation\\wall_type_1.ttm', '.'), ('C:\\Users\\Atul Sahay\\Desktop\\BM\\Task 6\\evaluation\\wall_type_2.ttm', '.'), ('C:\\Users\\Atul Sahay\\Desktop\\BM\\Task 6\\evaluation\\wall_type_3.ttm', '.')],
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
          name='test_task_6',
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
