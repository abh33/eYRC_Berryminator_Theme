# -*- mode: python ; coding: utf-8 -*-


block_cipher = None


a = Analysis(['test_task_2a.py'],
             pathex=['C:\\Users\\abhin\\Desktop\\Workspace\\Task 2A\\Pyinstaller'],
             binaries=[('C:\\Users\\abhin\\Desktop\\Workspace\\Task 2A\\Pyinstaller\\libiconv.dll', '.'), ('C:\\Users\\abhin\\Desktop\\Workspace\\Task 2A\\Pyinstaller\\libzbar-64.dll', '.'), ('C:\\Users\\abhin\\Desktop\\Workspace\\Task 2A\\Pyinstaller\\sim.py', '.'), ('C:\\Users\\abhin\\Desktop\\Workspace\\Task 2A\\Pyinstaller\\simConst.py', '.'), ('C:\\Users\\abhin\\Desktop\\Workspace\\Task 2A\\Pyinstaller\\remoteApi.dll', '.')],
             datas=[('C:\\Users\\abhin\\Desktop\\Workspace\\Task 2A\\Pyinstaller\\strawberry_fruit.ttm', '.'), ('C:\\Users\\abhin\\Desktop\\Workspace\\Task 2A\\Pyinstaller\\blueberry_fruit.ttm', '.'), ('C:\\Users\\abhin\\Desktop\\Workspace\\Task 2A\\Pyinstaller\\lemon_fruit.ttm', '.')],
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
          name='test_task_2a',
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
