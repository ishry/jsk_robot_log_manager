# Neuro VMログの読み方

- `neuro/neuro_7001/log_messages.log` が根元，`neuro/neuro_7002/log_messages.log` が先端の文章ログ．
- `rx fail … reason="poll timeout"` は，対象モータの応答を取れず時間切れで戻った記録．主に，立ち上げ直後にでるエラー．24 Vが入った状態でも出る．
- 同じ `waiting_motor` で `elapsed_ms` と `frames` が増え続け，`ack_missing=1` が続く場合は，緊急停止による24 V遮断を疑う．
- `ack_missing=0` は，その通知がACK欠落を示していないという意味．単発の通知だけで緊急停止とは判断しない．
