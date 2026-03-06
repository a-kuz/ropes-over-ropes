use serde::Deserialize;
use std::sync::{Arc, Mutex};

const BASE_URL: &str = "https://uzls-leaderboard.a-kuz.workers.dev";

#[derive(Deserialize, Clone, Default, Debug)]
pub struct StatsResponse {
    pub total_completions: i32,
    pub avg_moves: f64,
    pub best_moves: i32,
    pub player_best: Option<i32>,
    pub percentile: Option<i32>,
}

#[derive(Clone, Default)]
pub struct LeaderboardResult {
    pub stars: u32,
    pub percentile: Option<i32>,
    pub ready: bool,
}

pub struct Leaderboard {
    player_id: String,
    username: String,
    result: Arc<Mutex<LeaderboardResult>>,
}

impl Leaderboard {
    pub fn new() -> Self {
        let player_id = load_string("lb_player_id").unwrap_or_else(|| {
            let id = uuid_v4();
            save_string("lb_player_id", &id);
            id
        });
        let username = load_string("lb_username").unwrap_or_default();
        Self {
            player_id,
            username,
            result: Arc::new(Mutex::new(LeaderboardResult::default())),
        }
    }

    pub fn ensure_registered(&mut self) {
        if !self.username.is_empty() {
            return;
        }
        let body = format!(r#"{{"id":"{}"}}"#, self.player_id);
        let url = format!("{}/api/player", BASE_URL);
        #[cfg(not(target_arch = "wasm32"))]
        {
            if let Ok(mut resp) = ureq::post(&url)
                .header("Content-Type", "application/json")
                .send(&body)
            {
                if let Ok(text) = resp.body_mut().read_to_string() {
                    if let Ok(json) = serde_json::from_str::<serde_json::Value>(&text) {
                        if let Some(name) = json.get("username").and_then(|v| v.as_str()) {
                            self.username = name.to_string();
                            save_string("lb_username", &self.username);
                        }
                    }
                }
            }
        }
        #[cfg(target_arch = "wasm32")]
        {
            let _ = url;
            let _ = body;
        }
    }

    pub fn username(&self) -> &str {
        &self.username
    }

    pub fn set_username(&mut self, name: &str) {
        self.username = name.to_string();
        save_string("lb_username", name);
    }

    pub fn login_as(&mut self, name: &str) {
        self.player_id = uuid_v4();
        save_string("lb_player_id", &self.player_id);
        self.set_username(name);
    }

    pub fn result(&self) -> LeaderboardResult {
        self.result.lock().unwrap().clone()
    }

    pub fn reset_result(&self) {
        let mut r = self.result.lock().unwrap();
        *r = LeaderboardResult::default();
    }

    pub fn submit_and_fetch(&self, level: usize, moves: u32, time_ms: u32, is_new_record: bool) {
        let pid = self.player_id.clone();
        let uname = if self.username.is_empty() {
            "anon".to_string()
        } else {
            self.username.clone()
        };
        let result = self.result.clone();

        #[cfg(not(target_arch = "wasm32"))]
        {
            std::thread::spawn(move || {
                if is_new_record {
                    let body = format!(
                        r#"{{"player_id":"{}","username":"{}","level_id":{},"moves":{},"time_ms":{}}}"#,
                        pid, uname, level, moves, time_ms
                    );
                    let _ = ureq::post(&format!("{}/api/submit", BASE_URL))
                        .header("Content-Type", "application/json")
                        .send(&body);
                }

                let url = format!(
                    "{}/api/stats/{}?player_id={}&moves={}",
                    BASE_URL, level, pid, moves
                );
                if let Ok(mut resp) = ureq::get(&url).call() {
                    if let Ok(text) = resp.body_mut().read_to_string() {
                        if let Ok(stats) = serde_json::from_str::<StatsResponse>(&text) {
                            let mut r = result.lock().unwrap();
                            r.percentile = stats.percentile;
                            let best = stats.best_moves;
                            r.stars = if best <= 0 {
                                3
                            } else if moves as i32 <= best
                                || moves as i32 <= ((best as f64 * 1.5).ceil() as i32)
                            {
                                3
                            } else if moves as i32 > best * 5 {
                                1
                            } else {
                                2
                            };
                            r.ready = true;
                        }
                    }
                }
            });
        }

        #[cfg(target_arch = "wasm32")]
        {
            let result_clone = result.clone();
            wasm_bindgen_futures::spawn_local(async move {
                if is_new_record {
                    let body = format!(
                        r#"{{"player_id":"{}","username":"{}","level_id":{},"moves":{},"time_ms":{}}}"#,
                        pid, uname, level, moves, time_ms
                    );
                    let _ = wasm_fetch_post(&format!("{}/api/submit", BASE_URL), &body).await;
                }

                let url = format!(
                    "{}/api/stats/{}?player_id={}&moves={}",
                    BASE_URL, level, pid, moves
                );
                if let Some(text) = wasm_fetch_get(&url).await {
                    if let Ok(stats) = serde_json::from_str::<StatsResponse>(&text) {
                        let mut r = result_clone.lock().unwrap();
                        r.percentile = stats.percentile;
                        let best = stats.best_moves;
                        r.stars = if best <= 0 {
                            3
                        } else if moves as i32 <= best
                            || moves as i32 <= ((best as f64 * 1.5).ceil() as i32)
                        {
                            3
                        } else if moves as i32 > best * 5 {
                            1
                        } else {
                            2
                        };
                        r.ready = true;
                    }
                }
            });
        }
    }
}

#[cfg(target_arch = "wasm32")]
async fn wasm_fetch_post(url: &str, body: &str) -> Option<String> {
    use wasm_bindgen::JsCast;
    use wasm_bindgen_futures::JsFuture;

    let mut opts = web_sys::RequestInit::new();
    opts.method("POST");
    opts.body(Some(&wasm_bindgen::JsValue::from_str(body)));
    opts.mode(web_sys::RequestMode::Cors);

    let request = web_sys::Request::new_with_str_and_init(url, &opts).ok()?;
    request
        .headers()
        .set("Content-Type", "application/json")
        .ok()?;

    let window = web_sys::window()?;
    let resp_val = JsFuture::from(window.fetch_with_request(&request))
        .await
        .ok()?;
    let resp: web_sys::Response = resp_val.dyn_into().ok()?;
    let text = JsFuture::from(resp.text().ok()?).await.ok()?;
    text.as_string()
}

#[cfg(target_arch = "wasm32")]
async fn wasm_fetch_get(url: &str) -> Option<String> {
    use wasm_bindgen::JsCast;
    use wasm_bindgen_futures::JsFuture;

    let window = web_sys::window()?;
    let resp_val = JsFuture::from(window.fetch_with_str(url)).await.ok()?;
    let resp: web_sys::Response = resp_val.dyn_into().ok()?;
    let text = JsFuture::from(resp.text().ok()?).await.ok()?;
    text.as_string()
}

fn uuid_v4() -> String {
    use web_time::{SystemTime, UNIX_EPOCH};
    let t = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    format!("{:032x}", t ^ 0xdeadbeef_cafebabe_u128)
}

#[cfg(target_arch = "wasm32")]
fn save_string(key: &str, value: &str) {
    if let Some(storage) = web_sys::window().and_then(|w| w.local_storage().ok().flatten()) {
        let _ = storage.set_item(key, value);
    }
}

#[cfg(target_arch = "wasm32")]
fn load_string(key: &str) -> Option<String> {
    web_sys::window()
        .and_then(|w| w.local_storage().ok().flatten())
        .and_then(|s| s.get_item(key).ok().flatten())
        .filter(|s| !s.is_empty())
}

#[cfg(not(target_arch = "wasm32"))]
fn save_string(key: &str, value: &str) {
    if let Some(dir) = dirs::data_local_dir() {
        let path = dir.join("uzls4");
        let _ = std::fs::create_dir_all(&path);
        let _ = std::fs::write(path.join(key), value);
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn load_string(key: &str) -> Option<String> {
    dirs::data_local_dir()
        .map(|d| d.join("uzls4").join(key))
        .and_then(|p| std::fs::read_to_string(p).ok())
        .filter(|s| !s.is_empty())
}
