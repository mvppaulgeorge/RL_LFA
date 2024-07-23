// Benchmark "adder" written by ABC on Thu Jul 11 11:32:03 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n315, new_n317, new_n319, new_n321;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[10] ), .clkout(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  and002aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oab012aa1n02x4               g006(.a(new_n99), .b(new_n101), .c(new_n100), .out0(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norb02aa1n02x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  norp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  norb02aa1n02x5               g012(.a(new_n107), .b(new_n106), .out0(new_n108));
  nanp03aa1n02x5               g013(.a(new_n102), .b(new_n105), .c(new_n108), .o1(new_n109));
  aoi012aa1n02x5               g014(.a(new_n103), .b(new_n106), .c(new_n104), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nano23aa1n02x4               g019(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n115));
  norp02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  norp02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nano23aa1n02x4               g024(.a(new_n116), .b(new_n118), .c(new_n119), .d(new_n117), .out0(new_n120));
  nanp02aa1n02x5               g025(.a(new_n120), .b(new_n115), .o1(new_n121));
  and002aa1n02x5               g026(.a(\b[5] ), .b(\a[6] ), .o(new_n122));
  oab012aa1n02x4               g027(.a(new_n122), .b(new_n116), .c(new_n118), .out0(new_n123));
  oai022aa1n02x5               g028(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n124));
  aoi022aa1n02x5               g029(.a(new_n115), .b(new_n123), .c(new_n112), .d(new_n124), .o1(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n121), .c(new_n109), .d(new_n110), .o1(new_n126));
  xorc02aa1n02x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoi012aa1n02x5               g032(.a(new_n98), .b(new_n126), .c(new_n127), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g034(.clk(\a[9] ), .clkout(new_n130));
  160nm_ficinv00aa1n08x5       g035(.clk(\b[9] ), .clkout(new_n131));
  xroi22aa1d04x5               g036(.a(new_n130), .b(\b[8] ), .c(new_n131), .d(\a[10] ), .out0(new_n132));
  oao003aa1n02x5               g037(.a(new_n97), .b(new_n131), .c(new_n98), .carry(new_n133));
  norp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n133), .c(new_n126), .d(new_n132), .o1(new_n137));
  aoi112aa1n02x5               g042(.a(new_n136), .b(new_n133), .c(new_n126), .d(new_n132), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g044(.clk(new_n134), .clkout(new_n140));
  norp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n137), .c(new_n140), .out0(\s[12] ));
  nano23aa1n02x4               g049(.a(new_n134), .b(new_n141), .c(new_n142), .d(new_n135), .out0(new_n145));
  oai012aa1n02x5               g050(.a(new_n142), .b(new_n141), .c(new_n134), .o1(new_n146));
  aobi12aa1n02x5               g051(.a(new_n146), .b(new_n145), .c(new_n133), .out0(new_n147));
  nona23aa1n02x4               g052(.a(new_n142), .b(new_n135), .c(new_n134), .d(new_n141), .out0(new_n148));
  norb02aa1n02x5               g053(.a(new_n132), .b(new_n148), .out0(new_n149));
  nanp02aa1n02x5               g054(.a(new_n126), .b(new_n149), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n150), .b(new_n147), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n153), .b(new_n151), .c(new_n154), .o1(new_n155));
  xnrb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[14] ), .b(\a[15] ), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  oaoi03aa1n02x5               g064(.a(new_n97), .b(new_n131), .c(new_n98), .o1(new_n160));
  oai012aa1n02x5               g065(.a(new_n146), .b(new_n148), .c(new_n160), .o1(new_n161));
  norp02aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nano23aa1n02x4               g068(.a(new_n153), .b(new_n162), .c(new_n163), .d(new_n154), .out0(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n161), .c(new_n126), .d(new_n149), .o1(new_n165));
  oa0012aa1n02x5               g070(.a(new_n163), .b(new_n162), .c(new_n153), .o(new_n166));
  160nm_ficinv00aa1n08x5       g071(.clk(new_n166), .clkout(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n159), .b(new_n165), .c(new_n167), .out0(\s[15] ));
  nanp02aa1n02x5               g073(.a(new_n165), .b(new_n167), .o1(new_n169));
  norp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n157), .c(new_n169), .d(new_n159), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(new_n169), .b(new_n159), .o1(new_n174));
  nona22aa1n02x4               g079(.a(new_n174), .b(new_n172), .c(new_n157), .out0(new_n175));
  nanp02aa1n02x5               g080(.a(new_n175), .b(new_n173), .o1(\s[16] ));
  nano23aa1n02x4               g081(.a(new_n157), .b(new_n170), .c(new_n171), .d(new_n158), .out0(new_n177));
  nanp02aa1n02x5               g082(.a(new_n177), .b(new_n164), .o1(new_n178));
  nano22aa1n02x4               g083(.a(new_n178), .b(new_n132), .c(new_n145), .out0(new_n179));
  nanp02aa1n02x5               g084(.a(new_n126), .b(new_n179), .o1(new_n180));
  oai012aa1n02x5               g085(.a(new_n171), .b(new_n170), .c(new_n157), .o1(new_n181));
  aobi12aa1n02x5               g086(.a(new_n181), .b(new_n177), .c(new_n166), .out0(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(new_n182), .clkout(new_n183));
  aoib12aa1n02x5               g088(.a(new_n183), .b(new_n161), .c(new_n178), .out0(new_n184));
  xorc02aa1n02x5               g089(.a(\a[17] ), .b(\b[16] ), .out0(new_n185));
  xnbna2aa1n03x5               g090(.a(new_n185), .b(new_n180), .c(new_n184), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[18] ), .clkout(new_n187));
  nanp02aa1n02x5               g092(.a(new_n180), .b(new_n184), .o1(new_n188));
  norp02aa1n02x5               g093(.a(\b[16] ), .b(\a[17] ), .o1(new_n189));
  aoi012aa1n02x5               g094(.a(new_n189), .b(new_n188), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  oai012aa1n02x5               g096(.a(new_n182), .b(new_n147), .c(new_n178), .o1(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(\a[17] ), .clkout(new_n193));
  xroi22aa1d04x5               g098(.a(new_n193), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n194));
  aoai13aa1n02x5               g099(.a(new_n194), .b(new_n192), .c(new_n126), .d(new_n179), .o1(new_n195));
  160nm_ficinv00aa1n08x5       g100(.clk(\b[17] ), .clkout(new_n196));
  oaoi03aa1n02x5               g101(.a(new_n187), .b(new_n196), .c(new_n189), .o1(new_n197));
  norp02aa1n02x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(new_n200));
  160nm_ficinv00aa1n08x5       g105(.clk(new_n200), .clkout(new_n201));
  xnbna2aa1n03x5               g106(.a(new_n201), .b(new_n195), .c(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g108(.a(new_n195), .b(new_n197), .o1(new_n204));
  norp02aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanb02aa1n02x5               g111(.a(new_n205), .b(new_n206), .out0(new_n207));
  aoai13aa1n02x5               g112(.a(new_n207), .b(new_n198), .c(new_n204), .d(new_n201), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(new_n204), .b(new_n201), .o1(new_n209));
  nona22aa1n02x4               g114(.a(new_n209), .b(new_n207), .c(new_n198), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n210), .b(new_n208), .o1(\s[20] ));
  nona23aa1n02x4               g116(.a(new_n206), .b(new_n199), .c(new_n198), .d(new_n205), .out0(new_n212));
  oa0012aa1n02x5               g117(.a(new_n206), .b(new_n205), .c(new_n198), .o(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n213), .clkout(new_n214));
  oai012aa1n02x5               g119(.a(new_n214), .b(new_n212), .c(new_n197), .o1(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  nano23aa1n02x4               g121(.a(new_n198), .b(new_n205), .c(new_n206), .d(new_n199), .out0(new_n217));
  nanp02aa1n02x5               g122(.a(new_n194), .b(new_n217), .o1(new_n218));
  aoai13aa1n02x5               g123(.a(new_n216), .b(new_n218), .c(new_n180), .d(new_n184), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xorc02aa1n02x5               g127(.a(\a[22] ), .b(\b[21] ), .out0(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(new_n223), .clkout(new_n224));
  aoai13aa1n02x5               g129(.a(new_n224), .b(new_n221), .c(new_n219), .d(new_n222), .o1(new_n225));
  nanp02aa1n02x5               g130(.a(new_n219), .b(new_n222), .o1(new_n226));
  nona22aa1n02x4               g131(.a(new_n226), .b(new_n224), .c(new_n221), .out0(new_n227));
  nanp02aa1n02x5               g132(.a(new_n227), .b(new_n225), .o1(\s[22] ));
  160nm_ficinv00aa1n08x5       g133(.clk(\a[21] ), .clkout(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(\a[22] ), .clkout(new_n230));
  xroi22aa1d04x5               g135(.a(new_n229), .b(\b[20] ), .c(new_n230), .d(\b[21] ), .out0(new_n231));
  nanp03aa1n02x5               g136(.a(new_n231), .b(new_n194), .c(new_n217), .o1(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(\b[21] ), .clkout(new_n233));
  oao003aa1n02x5               g138(.a(new_n230), .b(new_n233), .c(new_n221), .carry(new_n234));
  aoi012aa1n02x5               g139(.a(new_n234), .b(new_n215), .c(new_n231), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n235), .b(new_n232), .c(new_n180), .d(new_n184), .o1(new_n236));
  xorb03aa1n02x5               g141(.a(new_n236), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  xorc02aa1n02x5               g143(.a(\a[23] ), .b(\b[22] ), .out0(new_n239));
  xnrc02aa1n02x5               g144(.a(\b[23] ), .b(\a[24] ), .out0(new_n240));
  aoai13aa1n02x5               g145(.a(new_n240), .b(new_n238), .c(new_n236), .d(new_n239), .o1(new_n241));
  nanp02aa1n02x5               g146(.a(new_n236), .b(new_n239), .o1(new_n242));
  nona22aa1n02x4               g147(.a(new_n242), .b(new_n240), .c(new_n238), .out0(new_n243));
  nanp02aa1n02x5               g148(.a(new_n243), .b(new_n241), .o1(\s[24] ));
  norb02aa1n02x5               g149(.a(new_n239), .b(new_n240), .out0(new_n245));
  nano22aa1n02x4               g150(.a(new_n218), .b(new_n245), .c(new_n231), .out0(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n192), .c(new_n126), .d(new_n179), .o1(new_n247));
  oao003aa1n02x5               g152(.a(new_n187), .b(new_n196), .c(new_n189), .carry(new_n248));
  aoai13aa1n02x5               g153(.a(new_n231), .b(new_n213), .c(new_n217), .d(new_n248), .o1(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n234), .clkout(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n245), .clkout(new_n251));
  orn002aa1n02x5               g156(.a(\a[23] ), .b(\b[22] ), .o(new_n252));
  oaoi03aa1n02x5               g157(.a(\a[24] ), .b(\b[23] ), .c(new_n252), .o1(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n253), .clkout(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n251), .c(new_n249), .d(new_n250), .o1(new_n255));
  nanb02aa1n02x5               g160(.a(new_n255), .b(new_n247), .out0(new_n256));
  xorb03aa1n02x5               g161(.a(new_n256), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  xorc02aa1n02x5               g163(.a(\a[25] ), .b(\b[24] ), .out0(new_n259));
  norp02aa1n02x5               g164(.a(\b[25] ), .b(\a[26] ), .o1(new_n260));
  nanp02aa1n02x5               g165(.a(\b[25] ), .b(\a[26] ), .o1(new_n261));
  norb02aa1n02x5               g166(.a(new_n261), .b(new_n260), .out0(new_n262));
  160nm_ficinv00aa1n08x5       g167(.clk(new_n262), .clkout(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n258), .c(new_n256), .d(new_n259), .o1(new_n264));
  aoai13aa1n02x5               g169(.a(new_n259), .b(new_n255), .c(new_n188), .d(new_n246), .o1(new_n265));
  nona22aa1n02x4               g170(.a(new_n265), .b(new_n263), .c(new_n258), .out0(new_n266));
  nanp02aa1n02x5               g171(.a(new_n264), .b(new_n266), .o1(\s[26] ));
  norb02aa1n02x5               g172(.a(new_n259), .b(new_n263), .out0(new_n268));
  nano22aa1n02x4               g173(.a(new_n232), .b(new_n245), .c(new_n268), .out0(new_n269));
  aoai13aa1n02x5               g174(.a(new_n269), .b(new_n192), .c(new_n126), .d(new_n179), .o1(new_n270));
  nanp02aa1n02x5               g175(.a(new_n255), .b(new_n268), .o1(new_n271));
  oai012aa1n02x5               g176(.a(new_n261), .b(new_n260), .c(new_n258), .o1(new_n272));
  nanp03aa1n02x5               g177(.a(new_n271), .b(new_n270), .c(new_n272), .o1(new_n273));
  xorb03aa1n02x5               g178(.a(new_n273), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  xorc02aa1n02x5               g180(.a(\a[27] ), .b(\b[26] ), .out0(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  aoai13aa1n02x5               g182(.a(new_n277), .b(new_n275), .c(new_n273), .d(new_n276), .o1(new_n278));
  aobi12aa1n02x5               g183(.a(new_n269), .b(new_n180), .c(new_n184), .out0(new_n279));
  aoai13aa1n02x5               g184(.a(new_n245), .b(new_n234), .c(new_n215), .d(new_n231), .o1(new_n280));
  160nm_ficinv00aa1n08x5       g185(.clk(new_n268), .clkout(new_n281));
  aoai13aa1n02x5               g186(.a(new_n272), .b(new_n281), .c(new_n280), .d(new_n254), .o1(new_n282));
  oai012aa1n02x5               g187(.a(new_n276), .b(new_n282), .c(new_n279), .o1(new_n283));
  nona22aa1n02x4               g188(.a(new_n283), .b(new_n277), .c(new_n275), .out0(new_n284));
  nanp02aa1n02x5               g189(.a(new_n278), .b(new_n284), .o1(\s[28] ));
  norb02aa1n02x5               g190(.a(new_n276), .b(new_n277), .out0(new_n286));
  oai012aa1n02x5               g191(.a(new_n286), .b(new_n282), .c(new_n279), .o1(new_n287));
  160nm_ficinv00aa1n08x5       g192(.clk(new_n275), .clkout(new_n288));
  oaoi03aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n288), .o1(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  nona22aa1n02x4               g195(.a(new_n287), .b(new_n289), .c(new_n290), .out0(new_n291));
  aoai13aa1n02x5               g196(.a(new_n290), .b(new_n289), .c(new_n273), .d(new_n286), .o1(new_n292));
  nanp02aa1n02x5               g197(.a(new_n292), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g198(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g199(.a(new_n276), .b(new_n290), .c(new_n277), .out0(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n288), .carry(new_n296));
  oaoi03aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .o1(new_n297));
  xorc02aa1n02x5               g202(.a(\a[30] ), .b(\b[29] ), .out0(new_n298));
  160nm_ficinv00aa1n08x5       g203(.clk(new_n298), .clkout(new_n299));
  aoai13aa1n02x5               g204(.a(new_n299), .b(new_n297), .c(new_n273), .d(new_n295), .o1(new_n300));
  oai012aa1n02x5               g205(.a(new_n295), .b(new_n282), .c(new_n279), .o1(new_n301));
  nona22aa1n02x4               g206(.a(new_n301), .b(new_n297), .c(new_n299), .out0(new_n302));
  nanp02aa1n02x5               g207(.a(new_n300), .b(new_n302), .o1(\s[30] ));
  nano23aa1n02x4               g208(.a(new_n290), .b(new_n277), .c(new_n298), .d(new_n276), .out0(new_n304));
  oai012aa1n02x5               g209(.a(new_n304), .b(new_n282), .c(new_n279), .o1(new_n305));
  nanp02aa1n02x5               g210(.a(new_n297), .b(new_n298), .o1(new_n306));
  oai012aa1n02x5               g211(.a(new_n306), .b(\b[29] ), .c(\a[30] ), .o1(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  nona22aa1n02x4               g213(.a(new_n305), .b(new_n307), .c(new_n308), .out0(new_n309));
  aoai13aa1n02x5               g214(.a(new_n308), .b(new_n307), .c(new_n273), .d(new_n304), .o1(new_n310));
  nanp02aa1n02x5               g215(.a(new_n310), .b(new_n309), .o1(\s[31] ));
  xorb03aa1n02x5               g216(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oai012aa1n02x5               g217(.a(new_n107), .b(new_n102), .c(new_n106), .o1(new_n313));
  xnrc02aa1n02x5               g218(.a(new_n313), .b(new_n105), .out0(\s[4] ));
  nanp02aa1n02x5               g219(.a(new_n109), .b(new_n110), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g221(.a(new_n118), .b(new_n315), .c(new_n119), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  160nm_fiao0012aa1n02p5x5     g223(.a(new_n123), .b(new_n315), .c(new_n120), .o(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g225(.a(new_n113), .b(new_n319), .c(new_n114), .o1(new_n321));
  xnrb03aa1n02x5               g226(.a(new_n321), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g227(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


