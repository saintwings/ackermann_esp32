"""
auth.py — Login / signup + robot-assignment RBAC for the robot control server.

Tables
------
  users        id, username, email, password, is_admin
  ws_tokens    token, user_id, expires_at        (bridges HTTP session → WebSocket)
  user_robots  user_id, robot_id                 (which user controls which robot)

Install deps:  pip install flask-login
"""

import os
import secrets
import sqlite3
import time

from flask import Blueprint, render_template, request, redirect, url_for, jsonify
from flask_login import (LoginManager, UserMixin,
                         login_user, logout_user, login_required, current_user)
from werkzeug.security import generate_password_hash, check_password_hash

# ── Paths ─────────────────────────────────────────────────────────────────────
_HERE           = os.path.dirname(os.path.abspath(__file__))
DB_PATH         = os.path.join(_HERE, 'users.db')
SECRET_KEY_FILE = os.path.join(_HERE, '.secret_key')
WS_TOKEN_TTL    = 86400   # 24 hours in seconds


# ── Secret key ────────────────────────────────────────────────────────────────
def get_secret_key() -> str:
    if os.path.exists(SECRET_KEY_FILE):
        with open(SECRET_KEY_FILE) as f:
            return f.read().strip()
    key = secrets.token_hex(32)
    with open(SECRET_KEY_FILE, 'w') as f:
        f.write(key)
    return key


# ── Database ──────────────────────────────────────────────────────────────────
def _init_db():
    with sqlite3.connect(DB_PATH) as conn:
        conn.executescript('''
            CREATE TABLE IF NOT EXISTS users (
                id       INTEGER PRIMARY KEY AUTOINCREMENT,
                username TEXT    UNIQUE NOT NULL,
                email    TEXT    UNIQUE NOT NULL,
                password TEXT    NOT NULL,
                is_admin INTEGER NOT NULL DEFAULT 0
            );

            CREATE TABLE IF NOT EXISTS ws_tokens (
                token      TEXT    PRIMARY KEY,
                user_id    INTEGER NOT NULL,
                expires_at REAL    NOT NULL
            );

            CREATE TABLE IF NOT EXISTS user_robots (
                user_id  INTEGER NOT NULL,
                robot_id TEXT    NOT NULL,
                PRIMARY KEY (user_id, robot_id)
            );
        ''')


def _db():
    return sqlite3.connect(DB_PATH)


# ── User queries ──────────────────────────────────────────────────────────────
def _find_by_id(user_id):
    with _db() as conn:
        return conn.execute(
            'SELECT id, username, email, is_admin FROM users WHERE id = ?',
            (user_id,)
        ).fetchone()


def _find_by_email(email):
    with _db() as conn:
        return conn.execute(
            'SELECT id, username, email, password, is_admin FROM users WHERE email = ?',
            (email,)
        ).fetchone()


def _user_count():
    with _db() as conn:
        return conn.execute('SELECT COUNT(*) FROM users').fetchone()[0]


def _create_user(username, email, password, is_admin=False) -> bool:
    try:
        with _db() as conn:
            conn.execute(
                'INSERT INTO users (username, email, password, is_admin) VALUES (?,?,?,?)',
                (username, email, generate_password_hash(password), int(is_admin))
            )
        return True
    except sqlite3.IntegrityError:
        return False


# ── User model ────────────────────────────────────────────────────────────────
class User(UserMixin):
    def __init__(self, id, username, email, is_admin=False):
        self.id       = str(id)
        self.username = username
        self.email    = email
        self.is_admin = bool(is_admin)


# ── WS Token ──────────────────────────────────────────────────────────────────
def create_ws_token(user_id: int) -> str:
    token = secrets.token_urlsafe(32)
    expires_at = time.time() + WS_TOKEN_TTL
    with _db() as conn:
        # Remove old tokens for this user
        conn.execute('DELETE FROM ws_tokens WHERE user_id = ?', (user_id,))
        conn.execute(
            'INSERT INTO ws_tokens (token, user_id, expires_at) VALUES (?,?,?)',
            (token, user_id, expires_at)
        )
    return token


def validate_ws_token(token: str):
    """Returns User if token is valid and not expired, else None."""
    with _db() as conn:
        row = conn.execute(
            'SELECT user_id, expires_at FROM ws_tokens WHERE token = ?', (token,)
        ).fetchone()
    if not row:
        return None
    user_id, expires_at = row
    if time.time() > expires_at:
        return None
    user_row = _find_by_id(user_id)
    return User(*user_row) if user_row else None


# ── Robot assignments ─────────────────────────────────────────────────────────
def get_user_robots(user_id: int) -> list:
    """Return list of robot_id strings assigned to this user."""
    with _db() as conn:
        rows = conn.execute(
            'SELECT robot_id FROM user_robots WHERE user_id = ?', (user_id,)
        ).fetchall()
    return [r[0] for r in rows]


def assign_robot(user_id: int, robot_id: str) -> bool:
    try:
        with _db() as conn:
            conn.execute(
                'INSERT OR IGNORE INTO user_robots (user_id, robot_id) VALUES (?,?)',
                (user_id, robot_id)
            )
        return True
    except Exception:
        return False


def unassign_robot(user_id: int, robot_id: str) -> bool:
    with _db() as conn:
        conn.execute(
            'DELETE FROM user_robots WHERE user_id = ? AND robot_id = ?',
            (user_id, robot_id)
        )
    return True


def is_admin(user_id: int) -> bool:
    row = _find_by_id(user_id)
    return bool(row[3]) if row else False


def list_users() -> list:
    with _db() as conn:
        rows = conn.execute(
            'SELECT id, username, email, is_admin FROM users ORDER BY id'
        ).fetchall()
    return [{'id': r[0], 'username': r[1], 'email': r[2], 'is_admin': bool(r[3])}
            for r in rows]


def get_all_assignments() -> dict:
    """Returns {user_id: [robot_id, ...]}"""
    with _db() as conn:
        rows = conn.execute('SELECT user_id, robot_id FROM user_robots').fetchall()
    result = {}
    for user_id, robot_id in rows:
        result.setdefault(user_id, []).append(robot_id)
    return result


# ── Flask-Login setup ─────────────────────────────────────────────────────────
def setup_auth(app):
    app.secret_key = get_secret_key()
    _init_db()

    login_manager = LoginManager(app)
    login_manager.login_view    = 'auth.login'
    login_manager.login_message = 'Please log in to access the mission planner.'

    @login_manager.user_loader
    def load_user(user_id):
        row = _find_by_id(user_id)
        return User(*row) if row else None


# ── Blueprint ─────────────────────────────────────────────────────────────────
auth_bp = Blueprint('auth', __name__)


@auth_bp.route('/login', methods=['GET', 'POST'])
def login():
    error = None
    if request.method == 'POST':
        email    = request.form.get('email', '').strip().lower()
        password = request.form.get('password', '')
        row      = _find_by_email(email)
        if row and check_password_hash(row[3], password):
            user = User(row[0], row[1], row[2], row[4])
            login_user(user, remember=True)
            return redirect(request.args.get('next') or '/')
        error = 'Invalid email or password.'
    return render_template('login.html', error=error)


@auth_bp.route('/signup', methods=['GET', 'POST'])
def signup():
    error = None
    if request.method == 'POST':
        username = request.form.get('username', '').strip()
        email    = request.form.get('email', '').strip().lower()
        password = request.form.get('password', '')
        confirm  = request.form.get('confirm', '')

        if not username or not email or not password:
            error = 'All fields are required.'
        elif len(password) < 8:
            error = 'Password must be at least 8 characters.'
        elif password != confirm:
            error = 'Passwords do not match.'
        else:
            first_user = (_user_count() == 0)   # first user becomes admin
            if _create_user(username, email, password, is_admin=first_user):
                return redirect(url_for('auth.login', registered=1))
            error = 'Email or username already registered.'

    return render_template('signup.html', error=error)


@auth_bp.route('/logout')
@login_required
def logout():
    logout_user()
    return redirect(url_for('auth.login'))


@auth_bp.route('/admin')
@login_required
def admin():
    if not current_user.is_admin:
        return redirect('/')
    return render_template('admin.html')


# ── Auth API (called by JS) ───────────────────────────────────────────────────
@auth_bp.route('/api/auth/ws-token')
@login_required
def api_ws_token():
    """Return a short-lived WebSocket auth token for the current user."""
    token = create_ws_token(int(current_user.id))
    return jsonify({'token': token})


@auth_bp.route('/api/auth/me')
@login_required
def api_me():
    return jsonify({
        'user_id':  int(current_user.id),
        'username': current_user.username,
        'is_admin': current_user.is_admin,
        'robots':   get_user_robots(int(current_user.id))
    })


# ── Admin API ─────────────────────────────────────────────────────────────────
def _require_admin():
    if not current_user.is_authenticated or not current_user.is_admin:
        return jsonify({'error': 'Admin only'}), 403
    return None


@auth_bp.route('/api/admin/data')
@login_required
def api_admin_data():
    err = _require_admin()
    if err:
        return err
    return jsonify({
        'users':       list_users(),
        'assignments': get_all_assignments()
    })


@auth_bp.route('/api/admin/assign', methods=['POST'])
@login_required
def api_assign():
    err = _require_admin()
    if err:
        return err
    data     = request.get_json()
    user_id  = int(data['user_id'])
    robot_id = str(data['robot_id'])
    assign_robot(user_id, robot_id)
    return jsonify({'ok': True})


@auth_bp.route('/api/admin/unassign', methods=['POST'])
@login_required
def api_unassign():
    err = _require_admin()
    if err:
        return err
    data     = request.get_json()
    user_id  = int(data['user_id'])
    robot_id = str(data['robot_id'])
    unassign_robot(user_id, robot_id)
    return jsonify({'ok': True})


@auth_bp.route('/api/admin/set-admin', methods=['POST'])
@login_required
def api_set_admin():
    err = _require_admin()
    if err:
        return err
    data     = request.get_json()
    user_id  = int(data['user_id'])
    value    = int(bool(data.get('is_admin', False)))
    with _db() as conn:
        conn.execute('UPDATE users SET is_admin = ? WHERE id = ?', (value, user_id))
    return jsonify({'ok': True})
