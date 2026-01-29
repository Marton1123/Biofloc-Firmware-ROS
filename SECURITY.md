# Guía de Seguridad

## Archivos con Información Sensible

Este proyecto maneja información sensible que **NUNCA debe ser commiteada** al repositorio:

### 🔒 Archivos Protegidos (en .gitignore)

1. **`sdkconfig`** - Configuración ESP32 con credenciales WiFi
   - Contiene: SSID y contraseña WiFi, IPs de Agent
   - Solución: Usar `sdkconfig.example` como template
   - Crear tu propio `sdkconfig` localmente con `idf.py menuconfig`

2. **`scripts/.env`** - Credenciales de MongoDB
   - Contiene: URI de MongoDB Atlas con usuario/contraseña
   - Solución: Usar `scripts/.env.example` como template
   - Copiar `.env.example` a `.env` y completar con tus credenciales

3. **Archivos de build/**
   - Contiene copias de configuración compiladas
   - Automáticamente ignorados

## ⚠️ Antes de Hacer Commit

**Checklist de seguridad:**

```bash
# 1. Verificar que sdkconfig está ignorado
git status | grep sdkconfig
# Debe decir: "modified: sdkconfig" (no staged)

# 2. Verificar que .env está ignorado
git status | grep .env
# No debe aparecer

# 3. Buscar credenciales en cambios staged
git diff --cached | grep -E "(password|PASSWORD|mongodb.*:|192\.168\.0\.)"
# No debe retornar nada

# 4. Verificar .gitignore activo
cat .gitignore | grep -E "^sdkconfig$|^\.env$"
# Debe mostrar ambas líneas sin comentar
```

## 🔧 Configuración Inicial Segura

### 1. WiFi y Agent (ESP32)

```bash
# Copiar template
cp sdkconfig.example sdkconfig

# Configurar credenciales
idf.py menuconfig
# Ir a: Biofloc Configuration -> WiFi Configuration
#   - WiFi SSID: TU_RED
#   - WiFi Password: TU_CONTRASEÑA
#   - Agent IP: IP_DE_TU_PC

# Compilar (sdkconfig NO se commitea)
idf.py build flash
```

### 2. MongoDB (Scripts Python)

```bash
cd scripts

# Copiar template
cp .env.example .env

# Editar con tus credenciales
nano .env
# MONGODB_URI=mongodb+srv://usuario:contraseña@cluster.mongodb.net/...

# Probar conexión
python3 check_mongodb.py
```

## 🚨 Si Accidentalmente Commiteaste Credenciales

### Opción 1: Commit reciente (no pusheado)

```bash
# Deshacer último commit (mantener cambios)
git reset HEAD~1

# Limpiar credenciales del archivo
# Editar sdkconfig o .env para remover contraseñas

# Rehacer commit sin credenciales
git add <archivos_seguros>
git commit -m "..."
```

### Opción 2: Ya hiciste push (CRÍTICO)

```bash
# 1. Cambiar inmediatamente todas las contraseñas expuestas
#    - WiFi: Cambiar contraseña del router
#    - MongoDB: Rotar credenciales en Atlas

# 2. Usar BFG Repo-Cleaner o git filter-branch
git filter-branch --force --index-filter \
  "git rm --cached --ignore-unmatch sdkconfig" \
  --prune-empty --tag-name-filter cat -- --all

# 3. Force push (PELIGROSO - coordinar con el equipo)
git push origin --force --all
```

### Opción 3: Repositorio público con secretos (EMERGENCIA)

1. **Rotar TODAS las credenciales inmediatamente**
2. **Eliminar el repositorio de GitHub**
3. **Crear nuevo repositorio limpio**
4. **Usar git-secrets o similar para prevención**

## 🛡️ Prevención

### Hooks de Git (Recomendado)

Crear `.git/hooks/pre-commit`:

```bash
#!/bin/bash
# Prevenir commit de credenciales

# Verificar si sdkconfig está en staging
if git diff --cached --name-only | grep -q "^sdkconfig$"; then
    echo "ERROR: No puedes commitear sdkconfig (contiene credenciales)"
    echo "Usar: git reset HEAD sdkconfig"
    exit 1
fi

# Buscar contraseñas en diff
if git diff --cached | grep -iE "password.*=.*[^X]" | grep -v "YOUR_WIFI_PASSWORD"; then
    echo "ADVERTENCIA: Se detectó posible contraseña en el commit"
    echo "Revisa cuidadosamente antes de proceder"
    read -p "¿Continuar? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi
```

Hacer ejecutable:
```bash
chmod +x .git/hooks/pre-commit
```

## 📚 Referencias

- [Git Secret Management](https://git-scm.com/book/en/v2/Git-Tools-Credential-Storage)
- [GitHub Security Best Practices](https://docs.github.com/en/code-security/getting-started/best-practices-for-preventing-data-leaks-in-your-organization)
- [ESP-IDF Configuration](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/kconfig.html)

## 🆘 Contacto de Seguridad

Si detectas credenciales expuestas en este repositorio:
1. **NO** hagas un issue público
2. Contacta al administrador del repositorio directamente
3. Incluye: commit hash, archivo, línea
