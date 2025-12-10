import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import ChatKitPortal from '@site/src/components/ChatKitPortal/ChatKitPortal';

if (ExecutionEnvironment.canUseDOM) {
  import('./chatkit-loader').then(({ initChatKit }) => {
    initChatKit();
  });
}