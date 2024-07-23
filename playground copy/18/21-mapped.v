// Benchmark "adder" written by ABC on Thu Jul 11 12:13:45 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n321, new_n324, new_n326, new_n327,
    new_n328, new_n329, new_n331, new_n332;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\a[9] ), .clkout(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\b[8] ), .clkout(new_n101));
  nanp02aa1n02x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  160nm_ficinv00aa1n08x5       g007(.clk(\a[2] ), .clkout(new_n103));
  160nm_ficinv00aa1n08x5       g008(.clk(\b[1] ), .clkout(new_n104));
  nanp02aa1n02x5               g009(.a(\b[0] ), .b(\a[1] ), .o1(new_n105));
  oaoi03aa1n02x5               g010(.a(new_n103), .b(new_n104), .c(new_n105), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nona23aa1n02x4               g015(.a(new_n110), .b(new_n108), .c(new_n107), .d(new_n109), .out0(new_n111));
  aoi012aa1n02x5               g016(.a(new_n107), .b(new_n109), .c(new_n108), .o1(new_n112));
  oai012aa1n02x5               g017(.a(new_n112), .b(new_n111), .c(new_n106), .o1(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .out0(new_n114));
  norp02aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nona23aa1n02x4               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  xnrc02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .out0(new_n120));
  norp03aa1n02x5               g025(.a(new_n119), .b(new_n120), .c(new_n114), .o1(new_n121));
  oai012aa1n02x5               g026(.a(new_n116), .b(new_n117), .c(new_n115), .o1(new_n122));
  orn002aa1n02x5               g027(.a(\a[5] ), .b(\b[4] ), .o(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[6] ), .b(\b[5] ), .c(new_n123), .o1(new_n124));
  oaib12aa1n02x5               g029(.a(new_n122), .b(new_n119), .c(new_n124), .out0(new_n125));
  xorc02aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n125), .c(new_n113), .d(new_n121), .o1(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n99), .b(new_n127), .c(new_n102), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g033(.clk(new_n97), .clkout(new_n129));
  160nm_ficinv00aa1n08x5       g034(.clk(new_n98), .clkout(new_n130));
  aoai13aa1n02x5               g035(.a(new_n129), .b(new_n130), .c(new_n127), .d(new_n102), .o1(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  norp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  aoi112aa1n02x5               g043(.a(new_n138), .b(new_n133), .c(new_n131), .d(new_n135), .o1(new_n139));
  aoai13aa1n02x5               g044(.a(new_n138), .b(new_n133), .c(new_n131), .d(new_n135), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(\s[12] ));
  oao003aa1n02x5               g046(.a(new_n103), .b(new_n104), .c(new_n105), .carry(new_n142));
  nano23aa1n02x4               g047(.a(new_n107), .b(new_n109), .c(new_n110), .d(new_n108), .out0(new_n143));
  aobi12aa1n02x5               g048(.a(new_n112), .b(new_n143), .c(new_n142), .out0(new_n144));
  nano23aa1n02x4               g049(.a(new_n115), .b(new_n117), .c(new_n118), .d(new_n116), .out0(new_n145));
  nona22aa1n02x4               g050(.a(new_n145), .b(new_n120), .c(new_n114), .out0(new_n146));
  aobi12aa1n02x5               g051(.a(new_n122), .b(new_n145), .c(new_n124), .out0(new_n147));
  oai012aa1n02x5               g052(.a(new_n147), .b(new_n144), .c(new_n146), .o1(new_n148));
  nona23aa1n02x4               g053(.a(new_n137), .b(new_n134), .c(new_n133), .d(new_n136), .out0(new_n149));
  nano22aa1n02x4               g054(.a(new_n149), .b(new_n126), .c(new_n99), .out0(new_n150));
  oa0012aa1n02x5               g055(.a(new_n137), .b(new_n136), .c(new_n133), .o(new_n151));
  aoai13aa1n02x5               g056(.a(new_n98), .b(new_n97), .c(new_n100), .d(new_n101), .o1(new_n152));
  oabi12aa1n02x5               g057(.a(new_n151), .b(new_n152), .c(new_n149), .out0(new_n153));
  aoi012aa1n02x5               g058(.a(new_n153), .b(new_n148), .c(new_n150), .o1(new_n154));
  xnrb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  oaoi03aa1n02x5               g060(.a(\a[13] ), .b(\b[12] ), .c(new_n154), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  aoai13aa1n02x5               g062(.a(new_n150), .b(new_n125), .c(new_n113), .d(new_n121), .o1(new_n158));
  160nm_ficinv00aa1n08x5       g063(.clk(new_n153), .clkout(new_n159));
  norp02aa1n02x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  norp02aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nona23aa1n02x4               g068(.a(new_n163), .b(new_n161), .c(new_n160), .d(new_n162), .out0(new_n164));
  oai012aa1n02x5               g069(.a(new_n163), .b(new_n162), .c(new_n160), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n164), .c(new_n158), .d(new_n159), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  xorc02aa1n02x5               g073(.a(\a[15] ), .b(\b[14] ), .out0(new_n169));
  xorc02aa1n02x5               g074(.a(\a[16] ), .b(\b[15] ), .out0(new_n170));
  aoi112aa1n02x5               g075(.a(new_n170), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n170), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(\s[16] ));
  nanp03aa1n02x5               g078(.a(new_n135), .b(new_n99), .c(new_n138), .o1(new_n174));
  nano23aa1n02x4               g079(.a(new_n160), .b(new_n162), .c(new_n163), .d(new_n161), .out0(new_n175));
  nanp02aa1n02x5               g080(.a(new_n170), .b(new_n169), .o1(new_n176));
  nano23aa1n02x4               g081(.a(new_n176), .b(new_n174), .c(new_n175), .d(new_n126), .out0(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n125), .c(new_n113), .d(new_n121), .o1(new_n178));
  xnrc02aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .out0(new_n179));
  xnrc02aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .out0(new_n180));
  norp03aa1n02x5               g085(.a(new_n164), .b(new_n180), .c(new_n179), .o1(new_n181));
  aoi112aa1n02x5               g086(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n182));
  norp02aa1n02x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  160nm_ficinv00aa1n08x5       g088(.clk(new_n183), .clkout(new_n184));
  oai013aa1n02x4               g089(.a(new_n184), .b(new_n179), .c(new_n180), .d(new_n165), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n185), .b(new_n182), .c(new_n153), .d(new_n181), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(new_n178), .b(new_n186), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g093(.clk(\a[18] ), .clkout(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(\a[17] ), .clkout(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(\b[16] ), .clkout(new_n191));
  oaoi03aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  xroi22aa1d04x5               g098(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(new_n194), .clkout(new_n195));
  oai022aa1n02x5               g100(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n196));
  oaib12aa1n02x5               g101(.a(new_n196), .b(new_n189), .c(\b[17] ), .out0(new_n197));
  aoai13aa1n02x5               g102(.a(new_n197), .b(new_n195), .c(new_n178), .d(new_n186), .o1(new_n198));
  xorb03aa1n02x5               g103(.a(new_n198), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  160nm_ficinv00aa1n08x5       g107(.clk(\b[19] ), .clkout(new_n203));
  nanb02aa1n02x5               g108(.a(\a[20] ), .b(new_n203), .out0(new_n204));
  nanp02aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  aoi122aa1n02x5               g110(.a(new_n201), .b(new_n204), .c(new_n205), .d(new_n198), .e(new_n202), .o1(new_n206));
  160nm_ficinv00aa1n08x5       g111(.clk(new_n201), .clkout(new_n207));
  nanb02aa1n02x5               g112(.a(new_n201), .b(new_n202), .out0(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(new_n208), .clkout(new_n209));
  nanp02aa1n02x5               g114(.a(new_n198), .b(new_n209), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(new_n204), .b(new_n205), .o1(new_n211));
  aoi012aa1n02x5               g116(.a(new_n211), .b(new_n210), .c(new_n207), .o1(new_n212));
  norp02aa1n02x5               g117(.a(new_n212), .b(new_n206), .o1(\s[20] ));
  norp02aa1n02x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nona23aa1n02x4               g119(.a(new_n205), .b(new_n202), .c(new_n201), .d(new_n214), .out0(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  nanp02aa1n02x5               g121(.a(new_n194), .b(new_n216), .o1(new_n217));
  nanp02aa1n02x5               g122(.a(new_n201), .b(new_n205), .o1(new_n218));
  norp03aa1n02x5               g123(.a(new_n197), .b(new_n208), .c(new_n211), .o1(new_n219));
  nano22aa1n02x4               g124(.a(new_n219), .b(new_n204), .c(new_n218), .out0(new_n220));
  aoai13aa1n02x5               g125(.a(new_n220), .b(new_n217), .c(new_n178), .d(new_n186), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[21] ), .b(\b[20] ), .out0(new_n224));
  xorc02aa1n02x5               g129(.a(\a[22] ), .b(\b[21] ), .out0(new_n225));
  aoi112aa1n02x5               g130(.a(new_n223), .b(new_n225), .c(new_n221), .d(new_n224), .o1(new_n226));
  aoai13aa1n02x5               g131(.a(new_n225), .b(new_n223), .c(new_n221), .d(new_n224), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n227), .b(new_n226), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g133(.clk(\a[21] ), .clkout(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(\a[22] ), .clkout(new_n230));
  xroi22aa1d04x5               g135(.a(new_n229), .b(\b[20] ), .c(new_n230), .d(\b[21] ), .out0(new_n231));
  nanp03aa1n02x5               g136(.a(new_n231), .b(new_n194), .c(new_n216), .o1(new_n232));
  oai112aa1n02x5               g137(.a(new_n218), .b(new_n204), .c(new_n215), .d(new_n197), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(\b[21] ), .clkout(new_n234));
  oaoi03aa1n02x5               g139(.a(new_n230), .b(new_n234), .c(new_n223), .o1(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n235), .clkout(new_n236));
  aoi012aa1n02x5               g141(.a(new_n236), .b(new_n233), .c(new_n231), .o1(new_n237));
  aoai13aa1n02x5               g142(.a(new_n237), .b(new_n232), .c(new_n178), .d(new_n186), .o1(new_n238));
  xorb03aa1n02x5               g143(.a(new_n238), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  xorc02aa1n02x5               g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  xorc02aa1n02x5               g146(.a(\a[24] ), .b(\b[23] ), .out0(new_n242));
  aoi112aa1n02x5               g147(.a(new_n240), .b(new_n242), .c(new_n238), .d(new_n241), .o1(new_n243));
  aoai13aa1n02x5               g148(.a(new_n242), .b(new_n240), .c(new_n238), .d(new_n241), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n244), .b(new_n243), .out0(\s[24] ));
  nanp02aa1n02x5               g150(.a(new_n225), .b(new_n224), .o1(new_n246));
  xnrc02aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .out0(new_n247));
  xnrc02aa1n02x5               g152(.a(\b[23] ), .b(\a[24] ), .out0(new_n248));
  norp02aa1n02x5               g153(.a(new_n248), .b(new_n247), .o1(new_n249));
  nona23aa1n02x4               g154(.a(new_n194), .b(new_n249), .c(new_n246), .d(new_n215), .out0(new_n250));
  nano22aa1n02x4               g155(.a(new_n246), .b(new_n241), .c(new_n242), .out0(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(\a[24] ), .clkout(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(\b[23] ), .clkout(new_n253));
  oaoi03aa1n02x5               g158(.a(new_n252), .b(new_n253), .c(new_n240), .o1(new_n254));
  oai013aa1n02x4               g159(.a(new_n254), .b(new_n235), .c(new_n247), .d(new_n248), .o1(new_n255));
  aoi012aa1n02x5               g160(.a(new_n255), .b(new_n233), .c(new_n251), .o1(new_n256));
  aoai13aa1n02x5               g161(.a(new_n256), .b(new_n250), .c(new_n178), .d(new_n186), .o1(new_n257));
  xorb03aa1n02x5               g162(.a(new_n257), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  xorc02aa1n02x5               g164(.a(\a[25] ), .b(\b[24] ), .out0(new_n260));
  xorc02aa1n02x5               g165(.a(\a[26] ), .b(\b[25] ), .out0(new_n261));
  aoi112aa1n02x5               g166(.a(new_n259), .b(new_n261), .c(new_n257), .d(new_n260), .o1(new_n262));
  aoai13aa1n02x5               g167(.a(new_n261), .b(new_n259), .c(new_n257), .d(new_n260), .o1(new_n263));
  norb02aa1n02x5               g168(.a(new_n263), .b(new_n262), .out0(\s[26] ));
  nanp02aa1n02x5               g169(.a(new_n153), .b(new_n181), .o1(new_n265));
  nona22aa1n02x4               g170(.a(new_n265), .b(new_n185), .c(new_n182), .out0(new_n266));
  and002aa1n02x5               g171(.a(new_n261), .b(new_n260), .o(new_n267));
  nano32aa1n02x4               g172(.a(new_n217), .b(new_n267), .c(new_n231), .d(new_n249), .out0(new_n268));
  aoai13aa1n02x5               g173(.a(new_n268), .b(new_n266), .c(new_n148), .d(new_n177), .o1(new_n269));
  norp02aa1n02x5               g174(.a(\b[25] ), .b(\a[26] ), .o1(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n270), .clkout(new_n271));
  aoi112aa1n02x5               g176(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n272), .clkout(new_n273));
  nanp02aa1n02x5               g178(.a(new_n231), .b(new_n249), .o1(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n255), .clkout(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n267), .clkout(new_n276));
  oaoi13aa1n02x5               g181(.a(new_n276), .b(new_n275), .c(new_n220), .d(new_n274), .o1(new_n277));
  nano22aa1n02x4               g182(.a(new_n277), .b(new_n271), .c(new_n273), .out0(new_n278));
  xorc02aa1n02x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  xnbna2aa1n03x5               g184(.a(new_n279), .b(new_n278), .c(new_n269), .out0(\s[27] ));
  norp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n281), .clkout(new_n282));
  aoai13aa1n02x5               g187(.a(new_n267), .b(new_n255), .c(new_n233), .d(new_n251), .o1(new_n283));
  nona22aa1n02x4               g188(.a(new_n283), .b(new_n272), .c(new_n270), .out0(new_n284));
  aoai13aa1n02x5               g189(.a(new_n279), .b(new_n284), .c(new_n187), .d(new_n268), .o1(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[27] ), .b(\a[28] ), .out0(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n285), .c(new_n282), .o1(new_n287));
  160nm_ficinv00aa1n08x5       g192(.clk(new_n279), .clkout(new_n288));
  aoi012aa1n02x5               g193(.a(new_n288), .b(new_n278), .c(new_n269), .o1(new_n289));
  nano22aa1n02x4               g194(.a(new_n289), .b(new_n282), .c(new_n286), .out0(new_n290));
  norp02aa1n02x5               g195(.a(new_n287), .b(new_n290), .o1(\s[28] ));
  norb02aa1n02x5               g196(.a(new_n279), .b(new_n286), .out0(new_n292));
  aoai13aa1n02x5               g197(.a(new_n292), .b(new_n284), .c(new_n187), .d(new_n268), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[28] ), .b(\a[29] ), .out0(new_n295));
  aoi012aa1n02x5               g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  160nm_ficinv00aa1n08x5       g201(.clk(new_n292), .clkout(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n278), .c(new_n269), .o1(new_n298));
  nano22aa1n02x4               g203(.a(new_n298), .b(new_n294), .c(new_n295), .out0(new_n299));
  norp02aa1n02x5               g204(.a(new_n296), .b(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g206(.a(new_n279), .b(new_n295), .c(new_n286), .out0(new_n302));
  aoai13aa1n02x5               g207(.a(new_n302), .b(new_n284), .c(new_n187), .d(new_n268), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n294), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[29] ), .b(\a[30] ), .out0(new_n305));
  aoi012aa1n02x5               g210(.a(new_n305), .b(new_n303), .c(new_n304), .o1(new_n306));
  160nm_ficinv00aa1n08x5       g211(.clk(new_n302), .clkout(new_n307));
  aoi012aa1n02x5               g212(.a(new_n307), .b(new_n278), .c(new_n269), .o1(new_n308));
  nano22aa1n02x4               g213(.a(new_n308), .b(new_n304), .c(new_n305), .out0(new_n309));
  norp02aa1n02x5               g214(.a(new_n306), .b(new_n309), .o1(\s[30] ));
  norb02aa1n02x5               g215(.a(new_n302), .b(new_n305), .out0(new_n311));
  160nm_ficinv00aa1n08x5       g216(.clk(new_n311), .clkout(new_n312));
  aoi012aa1n02x5               g217(.a(new_n312), .b(new_n278), .c(new_n269), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n304), .carry(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[30] ), .b(\a[31] ), .out0(new_n315));
  nano22aa1n02x4               g220(.a(new_n313), .b(new_n314), .c(new_n315), .out0(new_n316));
  aoai13aa1n02x5               g221(.a(new_n311), .b(new_n284), .c(new_n187), .d(new_n268), .o1(new_n317));
  aoi012aa1n02x5               g222(.a(new_n315), .b(new_n317), .c(new_n314), .o1(new_n318));
  norp02aa1n02x5               g223(.a(new_n318), .b(new_n316), .o1(\s[31] ));
  xnrb03aa1n02x5               g224(.a(new_n106), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g225(.a(\a[3] ), .b(\b[2] ), .c(new_n106), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g227(.a(new_n113), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g228(.a(\a[5] ), .b(\b[4] ), .c(new_n144), .o1(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  160nm_ficinv00aa1n08x5       g230(.clk(new_n114), .clkout(new_n326));
  norb02aa1n02x5               g231(.a(new_n118), .b(new_n117), .out0(new_n327));
  aoai13aa1n02x5               g232(.a(new_n327), .b(new_n124), .c(new_n324), .d(new_n326), .o1(new_n328));
  aoi112aa1n02x5               g233(.a(new_n124), .b(new_n327), .c(new_n324), .d(new_n326), .o1(new_n329));
  norb02aa1n02x5               g234(.a(new_n328), .b(new_n329), .out0(\s[7] ));
  norb02aa1n02x5               g235(.a(new_n116), .b(new_n115), .out0(new_n331));
  160nm_ficinv00aa1n08x5       g236(.clk(new_n117), .clkout(new_n332));
  xnbna2aa1n03x5               g237(.a(new_n331), .b(new_n328), .c(new_n332), .out0(\s[8] ));
  xorb03aa1n02x5               g238(.a(new_n148), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


