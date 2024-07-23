// Benchmark "adder" written by ABC on Thu Jul 11 13:00:07 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n314, new_n316, new_n318, new_n320,
    new_n322;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[9] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[8] ), .clkout(new_n98));
  and002aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oab012aa1n02x4               g006(.a(new_n99), .b(new_n101), .c(new_n100), .out0(new_n102));
  xorc02aa1n02x5               g007(.a(\a[4] ), .b(\b[3] ), .out0(new_n103));
  norp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  norb02aa1n02x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nanp03aa1n02x5               g011(.a(new_n102), .b(new_n103), .c(new_n106), .o1(new_n107));
  160nm_ficinv00aa1n08x5       g012(.clk(new_n104), .clkout(new_n108));
  oao003aa1n02x5               g013(.a(\a[4] ), .b(\b[3] ), .c(new_n108), .carry(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1n02x4               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  nanp02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nano23aa1n02x4               g023(.a(new_n117), .b(new_n116), .c(new_n118), .d(new_n115), .out0(new_n119));
  nanp02aa1n02x5               g024(.a(new_n119), .b(new_n114), .o1(new_n120));
  oa0012aa1n02x5               g025(.a(new_n115), .b(new_n116), .c(new_n117), .o(new_n121));
  oa0012aa1n02x5               g026(.a(new_n111), .b(new_n112), .c(new_n110), .o(new_n122));
  aoi012aa1n02x5               g027(.a(new_n122), .b(new_n114), .c(new_n121), .o1(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n120), .c(new_n107), .d(new_n109), .o1(new_n124));
  oao003aa1n02x5               g029(.a(new_n97), .b(new_n98), .c(new_n124), .carry(new_n125));
  xorb03aa1n02x5               g030(.a(new_n125), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nanp02aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norp02aa1n02x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  norp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[8] ), .b(\a[9] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(new_n98), .b(new_n97), .o1(new_n133));
  160nm_ficinv00aa1n08x5       g038(.clk(new_n133), .clkout(new_n134));
  oaoi13aa1n02x5               g039(.a(new_n131), .b(new_n132), .c(new_n124), .d(new_n134), .o1(new_n135));
  nano22aa1n02x4               g040(.a(new_n135), .b(new_n127), .c(new_n130), .out0(new_n136));
  oaoi13aa1n02x5               g041(.a(new_n130), .b(new_n127), .c(new_n125), .d(new_n131), .o1(new_n137));
  norp02aa1n02x5               g042(.a(new_n137), .b(new_n136), .o1(\s[11] ));
  norp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanb02aa1n02x5               g045(.a(new_n139), .b(new_n140), .out0(new_n141));
  oai012aa1n02x5               g046(.a(new_n141), .b(new_n136), .c(new_n128), .o1(new_n142));
  orn003aa1n02x5               g047(.a(new_n136), .b(new_n128), .c(new_n141), .o(new_n143));
  nanp02aa1n02x5               g048(.a(new_n143), .b(new_n142), .o1(\s[12] ));
  nona23aa1n02x4               g049(.a(new_n140), .b(new_n129), .c(new_n128), .d(new_n139), .out0(new_n145));
  aoai13aa1n02x5               g050(.a(new_n127), .b(new_n131), .c(new_n97), .d(new_n98), .o1(new_n146));
  oa0012aa1n02x5               g051(.a(new_n140), .b(new_n139), .c(new_n128), .o(new_n147));
  oabi12aa1n02x5               g052(.a(new_n147), .b(new_n145), .c(new_n146), .out0(new_n148));
  norb02aa1n02x5               g053(.a(new_n127), .b(new_n131), .out0(new_n149));
  nano23aa1n02x4               g054(.a(new_n128), .b(new_n139), .c(new_n140), .d(new_n129), .out0(new_n150));
  xorc02aa1n02x5               g055(.a(\a[9] ), .b(\b[8] ), .out0(new_n151));
  nanp03aa1n02x5               g056(.a(new_n150), .b(new_n149), .c(new_n151), .o1(new_n152));
  160nm_ficinv00aa1n08x5       g057(.clk(new_n152), .clkout(new_n153));
  xnrc02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .out0(new_n154));
  160nm_ficinv00aa1n08x5       g059(.clk(new_n154), .clkout(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n148), .c(new_n124), .d(new_n153), .o1(new_n156));
  aoi112aa1n02x5               g061(.a(new_n148), .b(new_n155), .c(new_n124), .d(new_n153), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n156), .b(new_n157), .out0(\s[13] ));
  norp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  160nm_ficinv00aa1n08x5       g064(.clk(new_n159), .clkout(new_n160));
  xnrc02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .out0(new_n161));
  xobna2aa1n03x5               g066(.a(new_n161), .b(new_n156), .c(new_n160), .out0(\s[14] ));
  norp02aa1n02x5               g067(.a(new_n161), .b(new_n154), .o1(new_n163));
  oaoi03aa1n02x5               g068(.a(\a[14] ), .b(\b[13] ), .c(new_n160), .o1(new_n164));
  aoi012aa1n02x5               g069(.a(new_n164), .b(new_n148), .c(new_n163), .o1(new_n165));
  nona32aa1n02x4               g070(.a(new_n124), .b(new_n161), .c(new_n154), .d(new_n152), .out0(new_n166));
  xnrc02aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .out0(new_n167));
  xobna2aa1n03x5               g072(.a(new_n167), .b(new_n166), .c(new_n165), .out0(\s[15] ));
  norp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  160nm_fiao0012aa1n02p5x5     g074(.a(new_n167), .b(new_n166), .c(new_n165), .o(new_n170));
  xnrc02aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .out0(new_n171));
  oaib12aa1n02x5               g076(.a(new_n171), .b(new_n169), .c(new_n170), .out0(new_n172));
  nona22aa1n02x4               g077(.a(new_n170), .b(new_n171), .c(new_n169), .out0(new_n173));
  nanp02aa1n02x5               g078(.a(new_n172), .b(new_n173), .o1(\s[16] ));
  160nm_ficinv00aa1n08x5       g079(.clk(\a[17] ), .clkout(new_n175));
  norp02aa1n02x5               g080(.a(new_n171), .b(new_n167), .o1(new_n176));
  nano22aa1n02x4               g081(.a(new_n152), .b(new_n163), .c(new_n176), .out0(new_n177));
  160nm_ficinv00aa1n08x5       g082(.clk(\a[16] ), .clkout(new_n178));
  160nm_ficinv00aa1n08x5       g083(.clk(\b[15] ), .clkout(new_n179));
  oao003aa1n02x5               g084(.a(new_n178), .b(new_n179), .c(new_n169), .carry(new_n180));
  oaoi03aa1n02x5               g085(.a(\a[10] ), .b(\b[9] ), .c(new_n133), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n163), .b(new_n147), .c(new_n150), .d(new_n181), .o1(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(new_n164), .clkout(new_n183));
  160nm_ficinv00aa1n08x5       g088(.clk(new_n176), .clkout(new_n184));
  aoi012aa1n02x5               g089(.a(new_n184), .b(new_n182), .c(new_n183), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n185), .b(new_n180), .c(new_n124), .d(new_n177), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(new_n175), .out0(\s[17] ));
  oaoi03aa1n02x5               g092(.a(\a[17] ), .b(\b[16] ), .c(new_n186), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g094(.clk(\a[18] ), .clkout(new_n190));
  xroi22aa1d04x5               g095(.a(new_n175), .b(\b[16] ), .c(new_n190), .d(\b[17] ), .out0(new_n191));
  160nm_ficinv00aa1n08x5       g096(.clk(new_n191), .clkout(new_n192));
  oai022aa1n02x5               g097(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n193));
  oaib12aa1n02x5               g098(.a(new_n193), .b(new_n190), .c(\b[17] ), .out0(new_n194));
  oai012aa1n02x5               g099(.a(new_n194), .b(new_n186), .c(new_n192), .o1(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g101(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(new_n200));
  160nm_ficinv00aa1n08x5       g105(.clk(new_n200), .clkout(new_n201));
  norp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  aoai13aa1n02x5               g109(.a(new_n204), .b(new_n198), .c(new_n195), .d(new_n201), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(new_n124), .b(new_n177), .o1(new_n206));
  160nm_ficinv00aa1n08x5       g111(.clk(new_n180), .clkout(new_n207));
  oai112aa1n02x5               g112(.a(new_n206), .b(new_n207), .c(new_n165), .d(new_n184), .o1(new_n208));
  nanb02aa1n02x5               g113(.a(\b[16] ), .b(new_n175), .out0(new_n209));
  oaoi03aa1n02x5               g114(.a(\a[18] ), .b(\b[17] ), .c(new_n209), .o1(new_n210));
  aoai13aa1n02x5               g115(.a(new_n201), .b(new_n210), .c(new_n208), .d(new_n191), .o1(new_n211));
  nona22aa1n02x4               g116(.a(new_n211), .b(new_n204), .c(new_n198), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(new_n205), .b(new_n212), .o1(\s[20] ));
  nona23aa1n02x4               g118(.a(new_n203), .b(new_n199), .c(new_n198), .d(new_n202), .out0(new_n214));
  oa0012aa1n02x5               g119(.a(new_n203), .b(new_n202), .c(new_n198), .o(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  oai012aa1n02x5               g121(.a(new_n216), .b(new_n214), .c(new_n194), .o1(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n217), .clkout(new_n218));
  nano23aa1n02x4               g123(.a(new_n198), .b(new_n202), .c(new_n203), .d(new_n199), .out0(new_n219));
  nanp02aa1n02x5               g124(.a(new_n191), .b(new_n219), .o1(new_n220));
  oai012aa1n02x5               g125(.a(new_n218), .b(new_n186), .c(new_n220), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  xnrc02aa1n02x5               g128(.a(\b[20] ), .b(\a[21] ), .out0(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(new_n224), .clkout(new_n225));
  xnrc02aa1n02x5               g130(.a(\b[21] ), .b(\a[22] ), .out0(new_n226));
  aoai13aa1n02x5               g131(.a(new_n226), .b(new_n223), .c(new_n221), .d(new_n225), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n220), .clkout(new_n228));
  aoai13aa1n02x5               g133(.a(new_n225), .b(new_n217), .c(new_n208), .d(new_n228), .o1(new_n229));
  nona22aa1n02x4               g134(.a(new_n229), .b(new_n226), .c(new_n223), .out0(new_n230));
  nanp02aa1n02x5               g135(.a(new_n227), .b(new_n230), .o1(\s[22] ));
  norp02aa1n02x5               g136(.a(new_n226), .b(new_n224), .o1(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(\a[22] ), .clkout(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(\b[21] ), .clkout(new_n234));
  oao003aa1n02x5               g139(.a(new_n233), .b(new_n234), .c(new_n223), .carry(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n217), .c(new_n232), .o1(new_n236));
  nano22aa1n02x4               g141(.a(new_n192), .b(new_n232), .c(new_n219), .out0(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n237), .clkout(new_n238));
  oai012aa1n02x5               g143(.a(new_n236), .b(new_n186), .c(new_n238), .o1(new_n239));
  xorb03aa1n02x5               g144(.a(new_n239), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  xorc02aa1n02x5               g146(.a(\a[23] ), .b(\b[22] ), .out0(new_n242));
  xnrc02aa1n02x5               g147(.a(\b[23] ), .b(\a[24] ), .out0(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n241), .c(new_n239), .d(new_n242), .o1(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n236), .clkout(new_n245));
  aoai13aa1n02x5               g150(.a(new_n242), .b(new_n245), .c(new_n208), .d(new_n237), .o1(new_n246));
  nona22aa1n02x4               g151(.a(new_n246), .b(new_n243), .c(new_n241), .out0(new_n247));
  nanp02aa1n02x5               g152(.a(new_n244), .b(new_n247), .o1(\s[24] ));
  norb02aa1n02x5               g153(.a(new_n242), .b(new_n243), .out0(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n249), .clkout(new_n250));
  nano32aa1n02x4               g155(.a(new_n250), .b(new_n191), .c(new_n232), .d(new_n219), .out0(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(new_n251), .clkout(new_n252));
  aoai13aa1n02x5               g157(.a(new_n232), .b(new_n215), .c(new_n219), .d(new_n210), .o1(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n235), .clkout(new_n254));
  oai022aa1n02x5               g159(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n255));
  aob012aa1n02x5               g160(.a(new_n255), .b(\b[23] ), .c(\a[24] ), .out0(new_n256));
  aoai13aa1n02x5               g161(.a(new_n256), .b(new_n250), .c(new_n253), .d(new_n254), .o1(new_n257));
  160nm_ficinv00aa1n08x5       g162(.clk(new_n257), .clkout(new_n258));
  oai012aa1n02x5               g163(.a(new_n258), .b(new_n186), .c(new_n252), .o1(new_n259));
  xorb03aa1n02x5               g164(.a(new_n259), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  xorc02aa1n02x5               g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  xnrc02aa1n02x5               g167(.a(\b[25] ), .b(\a[26] ), .out0(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n261), .c(new_n259), .d(new_n262), .o1(new_n264));
  aoai13aa1n02x5               g169(.a(new_n262), .b(new_n257), .c(new_n208), .d(new_n251), .o1(new_n265));
  nona22aa1n02x4               g170(.a(new_n265), .b(new_n263), .c(new_n261), .out0(new_n266));
  nanp02aa1n02x5               g171(.a(new_n264), .b(new_n266), .o1(\s[26] ));
  norb02aa1n02x5               g172(.a(new_n262), .b(new_n263), .out0(new_n268));
  nano23aa1n02x4               g173(.a(new_n220), .b(new_n250), .c(new_n268), .d(new_n232), .out0(new_n269));
  160nm_ficinv00aa1n08x5       g174(.clk(new_n269), .clkout(new_n270));
  nanp02aa1n02x5               g175(.a(\b[25] ), .b(\a[26] ), .o1(new_n271));
  oai022aa1n02x5               g176(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n272));
  aoi022aa1n02x5               g177(.a(new_n257), .b(new_n268), .c(new_n271), .d(new_n272), .o1(new_n273));
  oai012aa1n02x5               g178(.a(new_n273), .b(new_n186), .c(new_n270), .o1(new_n274));
  xorb03aa1n02x5               g179(.a(new_n274), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  xorc02aa1n02x5               g181(.a(\a[27] ), .b(\b[26] ), .out0(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[27] ), .b(\a[28] ), .out0(new_n278));
  aoai13aa1n02x5               g183(.a(new_n278), .b(new_n276), .c(new_n274), .d(new_n277), .o1(new_n279));
  aoai13aa1n02x5               g184(.a(new_n249), .b(new_n235), .c(new_n217), .d(new_n232), .o1(new_n280));
  160nm_ficinv00aa1n08x5       g185(.clk(new_n268), .clkout(new_n281));
  nanp02aa1n02x5               g186(.a(new_n272), .b(new_n271), .o1(new_n282));
  aoai13aa1n02x5               g187(.a(new_n282), .b(new_n281), .c(new_n280), .d(new_n256), .o1(new_n283));
  aoai13aa1n02x5               g188(.a(new_n277), .b(new_n283), .c(new_n208), .d(new_n269), .o1(new_n284));
  nona22aa1n02x4               g189(.a(new_n284), .b(new_n278), .c(new_n276), .out0(new_n285));
  nanp02aa1n02x5               g190(.a(new_n279), .b(new_n285), .o1(\s[28] ));
  norb02aa1n02x5               g191(.a(new_n277), .b(new_n278), .out0(new_n287));
  aoai13aa1n02x5               g192(.a(new_n287), .b(new_n283), .c(new_n208), .d(new_n269), .o1(new_n288));
  160nm_ficinv00aa1n08x5       g193(.clk(new_n276), .clkout(new_n289));
  oaoi03aa1n02x5               g194(.a(\a[28] ), .b(\b[27] ), .c(new_n289), .o1(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[28] ), .b(\a[29] ), .out0(new_n291));
  nona22aa1n02x4               g196(.a(new_n288), .b(new_n290), .c(new_n291), .out0(new_n292));
  aoai13aa1n02x5               g197(.a(new_n291), .b(new_n290), .c(new_n274), .d(new_n287), .o1(new_n293));
  nanp02aa1n02x5               g198(.a(new_n293), .b(new_n292), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g200(.a(new_n277), .b(new_n291), .c(new_n278), .out0(new_n296));
  oao003aa1n02x5               g201(.a(\a[28] ), .b(\b[27] ), .c(new_n289), .carry(new_n297));
  oaoi03aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n297), .o1(new_n298));
  xorc02aa1n02x5               g203(.a(\a[30] ), .b(\b[29] ), .out0(new_n299));
  160nm_ficinv00aa1n08x5       g204(.clk(new_n299), .clkout(new_n300));
  aoai13aa1n02x5               g205(.a(new_n300), .b(new_n298), .c(new_n274), .d(new_n296), .o1(new_n301));
  aoai13aa1n02x5               g206(.a(new_n296), .b(new_n283), .c(new_n208), .d(new_n269), .o1(new_n302));
  nona22aa1n02x4               g207(.a(new_n302), .b(new_n298), .c(new_n300), .out0(new_n303));
  nanp02aa1n02x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  nano23aa1n02x4               g209(.a(new_n291), .b(new_n278), .c(new_n299), .d(new_n277), .out0(new_n305));
  aoai13aa1n02x5               g210(.a(new_n305), .b(new_n283), .c(new_n208), .d(new_n269), .o1(new_n306));
  nanp02aa1n02x5               g211(.a(new_n298), .b(new_n299), .o1(new_n307));
  oai012aa1n02x5               g212(.a(new_n307), .b(\b[29] ), .c(\a[30] ), .o1(new_n308));
  xnrc02aa1n02x5               g213(.a(\b[30] ), .b(\a[31] ), .out0(new_n309));
  nona22aa1n02x4               g214(.a(new_n306), .b(new_n308), .c(new_n309), .out0(new_n310));
  aoai13aa1n02x5               g215(.a(new_n309), .b(new_n308), .c(new_n274), .d(new_n305), .o1(new_n311));
  nanp02aa1n02x5               g216(.a(new_n311), .b(new_n310), .o1(\s[31] ));
  xobna2aa1n03x5               g217(.a(new_n102), .b(new_n105), .c(new_n108), .out0(\s[3] ));
  oai012aa1n02x5               g218(.a(new_n105), .b(new_n102), .c(new_n104), .o1(new_n314));
  xnrb03aa1n02x5               g219(.a(new_n314), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  nanp02aa1n02x5               g220(.a(new_n107), .b(new_n109), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g222(.a(new_n117), .b(new_n316), .c(new_n118), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  160nm_fiao0012aa1n02p5x5     g224(.a(new_n121), .b(new_n316), .c(new_n119), .o(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g226(.a(new_n112), .b(new_n320), .c(new_n113), .o1(new_n322));
  xnrb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g228(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


