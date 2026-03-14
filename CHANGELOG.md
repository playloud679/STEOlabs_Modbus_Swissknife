## Changelog

### Unreleased

- semplificato l'output seriale di `scan` e `watch` per renderlo piu` leggibile durante il reverse engineering
- aggiunto il comando `auto` per eseguire scan, dump baseline, finestra guidata di stimolazione e watch finale
- migliorato `analyze` per mostrare i principali byte order a 32 bit (`ABCD`, `BADC`, `CDAB`, `DCBA`)
- introdotto il rilevamento guidato dei registri che cambiano durante una perturbazione reale del sensore
- aggiunto il watch finale dei registri piu` significativi emersi durante la stimolazione
- corrette diverse euristiche di ranking per distinguere meglio:
  - registri con baseline vicina a zero
  - registri wrapped o signed vicini a `0xFFFF`
  - registri rumorosi che collassano spesso a zero
- reso il calcolo del delta consapevole del wrap a 16 bit, per evitare che salti come `0xFFxx -> 0x01xx` vengano interpretati come escursioni gigantesche
- aggiornato il `README` con il flusso operativo reale del firmware e i comandi correnti
