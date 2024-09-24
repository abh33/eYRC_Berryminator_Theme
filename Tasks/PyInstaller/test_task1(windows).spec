# -*- mode: python ; coding: utf-8 -*-


block_cipher = None


a = Analysis(['test_task1.py'],
             pathex=['C:\\Users\\abhin\\Desktop\\Workspace'],
             binaries=[('C:\\Users\\abhin\\Desktop\\Workspace\\all_test_images\\*.png','all_test_images'),
             ('C:\\Users\\abhin\\Desktop\\Workspace\\task_1a_solutions.json', '.'), ('C:\\Users\\abhin\\Desktop\\Workspace\\task_1b_solutions.json', '.'), ('C:\\Users\\abhin\\Desktop\\Workspace\\testcases_1b.json', '.'),
             ('C:\\Users\\abhin\\Desktop\\Workspace\\libiconv.dll', '.'), ('C:\\Users\\abhin\\Desktop\\Workspace\\libzbar-64.dll', '.')],
             datas=[],
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
          name='test_task1',
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
