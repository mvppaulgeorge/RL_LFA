// Benchmark "adder" written by ABC on Wed Jul 10 17:17:26 2024

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
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n192, new_n193, new_n194, new_n195,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n311, new_n314, new_n315, new_n317, new_n319;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[10] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\a[9] ), .clkout(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\b[8] ), .clkout(new_n99));
  norp02aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nona23aa1n02x4               g008(.a(new_n103), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n104));
  xnrc02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .out0(new_n105));
  xnrc02aa1n02x5               g010(.a(\b[4] ), .b(\a[5] ), .out0(new_n106));
  norp03aa1n02x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nona23aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  nanp02aa1n02x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[0] ), .b(\a[1] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  oai012aa1n02x5               g020(.a(new_n113), .b(new_n115), .c(new_n114), .o1(new_n116));
  oa0012aa1n02x5               g021(.a(new_n109), .b(new_n110), .c(new_n108), .o(new_n117));
  oabi12aa1n02x5               g022(.a(new_n117), .b(new_n112), .c(new_n116), .out0(new_n118));
  nanp02aa1n02x5               g023(.a(new_n118), .b(new_n107), .o1(new_n119));
  oai022aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  aob012aa1n02x5               g025(.a(new_n120), .b(\b[5] ), .c(\a[6] ), .out0(new_n121));
  norp02aa1n02x5               g026(.a(new_n104), .b(new_n121), .o1(new_n122));
  aoi112aa1n02x5               g027(.a(new_n122), .b(new_n100), .c(new_n101), .d(new_n102), .o1(new_n123));
  nanp02aa1n02x5               g028(.a(new_n119), .b(new_n123), .o1(new_n124));
  oaoi03aa1n02x5               g029(.a(new_n98), .b(new_n99), .c(new_n124), .o1(new_n125));
  xorb03aa1n02x5               g030(.a(new_n125), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g031(.clk(\b[9] ), .clkout(new_n127));
  nanp02aa1n02x5               g032(.a(new_n127), .b(new_n97), .o1(new_n128));
  and002aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o(new_n129));
  norp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  160nm_ficinv00aa1n08x5       g037(.clk(new_n132), .clkout(new_n133));
  aoi112aa1n02x5               g038(.a(new_n133), .b(new_n129), .c(new_n125), .d(new_n128), .o1(new_n134));
  oaib12aa1n02x5               g039(.a(new_n124), .b(new_n99), .c(\a[9] ), .out0(new_n135));
  oai112aa1n02x5               g040(.a(new_n135), .b(new_n128), .c(\b[8] ), .d(\a[9] ), .o1(new_n136));
  oaoi13aa1n02x5               g041(.a(new_n132), .b(new_n136), .c(new_n97), .d(new_n127), .o1(new_n137));
  norp02aa1n02x5               g042(.a(new_n137), .b(new_n134), .o1(\s[11] ));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n130), .clkout(new_n139));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(new_n142));
  nano22aa1n02x4               g047(.a(new_n134), .b(new_n139), .c(new_n142), .out0(new_n143));
  nona22aa1n02x4               g048(.a(new_n136), .b(new_n133), .c(new_n129), .out0(new_n144));
  aoi012aa1n02x5               g049(.a(new_n142), .b(new_n144), .c(new_n139), .o1(new_n145));
  norp02aa1n02x5               g050(.a(new_n145), .b(new_n143), .o1(\s[12] ));
  xorc02aa1n02x5               g051(.a(\a[10] ), .b(\b[9] ), .out0(new_n147));
  xorc02aa1n02x5               g052(.a(\a[9] ), .b(\b[8] ), .out0(new_n148));
  nano23aa1n02x4               g053(.a(new_n130), .b(new_n140), .c(new_n141), .d(new_n131), .out0(new_n149));
  nanp03aa1n02x5               g054(.a(new_n149), .b(new_n147), .c(new_n148), .o1(new_n150));
  160nm_ficinv00aa1n08x5       g055(.clk(new_n150), .clkout(new_n151));
  nona23aa1n02x4               g056(.a(new_n141), .b(new_n131), .c(new_n130), .d(new_n140), .out0(new_n152));
  oai012aa1n02x5               g057(.a(new_n141), .b(new_n140), .c(new_n130), .o1(new_n153));
  oai022aa1n02x5               g058(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n154));
  oaib12aa1n02x5               g059(.a(new_n154), .b(new_n127), .c(\a[10] ), .out0(new_n155));
  oai012aa1n02x5               g060(.a(new_n153), .b(new_n152), .c(new_n155), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n156), .b(new_n124), .c(new_n151), .o1(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  oaoi03aa1n02x5               g063(.a(\a[13] ), .b(\b[12] ), .c(new_n157), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nanp02aa1n02x5               g065(.a(new_n102), .b(new_n101), .o1(new_n161));
  oai122aa1n02x7               g066(.a(new_n161), .b(new_n104), .c(new_n121), .d(\b[7] ), .e(\a[8] ), .o1(new_n162));
  aoai13aa1n02x5               g067(.a(new_n151), .b(new_n162), .c(new_n118), .d(new_n107), .o1(new_n163));
  160nm_ficinv00aa1n08x5       g068(.clk(new_n156), .clkout(new_n164));
  norp02aa1n02x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  norp02aa1n02x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nona23aa1n02x4               g073(.a(new_n168), .b(new_n166), .c(new_n165), .d(new_n167), .out0(new_n169));
  oai012aa1n02x5               g074(.a(new_n168), .b(new_n167), .c(new_n165), .o1(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n169), .c(new_n163), .d(new_n164), .o1(new_n171));
  xorb03aa1n02x5               g076(.a(new_n171), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  xnrc02aa1n02x5               g078(.a(\b[14] ), .b(\a[15] ), .out0(new_n174));
  160nm_ficinv00aa1n08x5       g079(.clk(new_n174), .clkout(new_n175));
  xnrc02aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .out0(new_n176));
  160nm_ficinv00aa1n08x5       g081(.clk(new_n176), .clkout(new_n177));
  aoi112aa1n02x5               g082(.a(new_n177), .b(new_n173), .c(new_n171), .d(new_n175), .o1(new_n178));
  aoai13aa1n02x5               g083(.a(new_n177), .b(new_n173), .c(new_n171), .d(new_n175), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(\s[16] ));
  nano23aa1n02x4               g085(.a(new_n165), .b(new_n167), .c(new_n168), .d(new_n166), .out0(new_n181));
  nano32aa1n02x4               g086(.a(new_n150), .b(new_n177), .c(new_n175), .d(new_n181), .out0(new_n182));
  aoai13aa1n02x5               g087(.a(new_n182), .b(new_n162), .c(new_n118), .d(new_n107), .o1(new_n183));
  norp03aa1n02x5               g088(.a(new_n169), .b(new_n176), .c(new_n174), .o1(new_n184));
  aoi112aa1n02x5               g089(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n185));
  norp02aa1n02x5               g090(.a(\b[15] ), .b(\a[16] ), .o1(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(new_n186), .clkout(new_n187));
  oai013aa1n02x4               g092(.a(new_n187), .b(new_n174), .c(new_n176), .d(new_n170), .o1(new_n188));
  aoi112aa1n02x5               g093(.a(new_n188), .b(new_n185), .c(new_n156), .d(new_n184), .o1(new_n189));
  nanp02aa1n02x5               g094(.a(new_n183), .b(new_n189), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g096(.clk(\a[18] ), .clkout(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(\a[17] ), .clkout(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(\b[16] ), .clkout(new_n194));
  oaoi03aa1n02x5               g099(.a(new_n193), .b(new_n194), .c(new_n190), .o1(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[17] ), .c(new_n192), .out0(\s[18] ));
  xroi22aa1d04x5               g101(.a(new_n193), .b(\b[16] ), .c(new_n192), .d(\b[17] ), .out0(new_n197));
  nanp02aa1n02x5               g102(.a(new_n194), .b(new_n193), .o1(new_n198));
  oaoi03aa1n02x5               g103(.a(\a[18] ), .b(\b[17] ), .c(new_n198), .o1(new_n199));
  norp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  aoai13aa1n02x5               g107(.a(new_n202), .b(new_n199), .c(new_n190), .d(new_n197), .o1(new_n203));
  aoi112aa1n02x5               g108(.a(new_n202), .b(new_n199), .c(new_n190), .d(new_n197), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nanp02aa1n02x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  nona22aa1n02x4               g114(.a(new_n203), .b(new_n209), .c(new_n200), .out0(new_n210));
  160nm_ficinv00aa1n08x5       g115(.clk(new_n209), .clkout(new_n211));
  oaoi13aa1n02x5               g116(.a(new_n211), .b(new_n203), .c(\a[19] ), .d(\b[18] ), .o1(new_n212));
  norb02aa1n02x5               g117(.a(new_n210), .b(new_n212), .out0(\s[20] ));
  nano23aa1n02x4               g118(.a(new_n200), .b(new_n207), .c(new_n208), .d(new_n201), .out0(new_n214));
  nanp02aa1n02x5               g119(.a(new_n197), .b(new_n214), .o1(new_n215));
  oai022aa1n02x5               g120(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n216));
  oaib12aa1n02x5               g121(.a(new_n216), .b(new_n192), .c(\b[17] ), .out0(new_n217));
  nona23aa1n02x4               g122(.a(new_n208), .b(new_n201), .c(new_n200), .d(new_n207), .out0(new_n218));
  aoi012aa1n02x5               g123(.a(new_n207), .b(new_n200), .c(new_n208), .o1(new_n219));
  oai012aa1n02x5               g124(.a(new_n219), .b(new_n218), .c(new_n217), .o1(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(new_n220), .clkout(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n215), .c(new_n183), .d(new_n189), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  xorc02aa1n02x5               g129(.a(\a[21] ), .b(\b[20] ), .out0(new_n225));
  xorc02aa1n02x5               g130(.a(\a[22] ), .b(\b[21] ), .out0(new_n226));
  aoi112aa1n02x5               g131(.a(new_n224), .b(new_n226), .c(new_n222), .d(new_n225), .o1(new_n227));
  aoai13aa1n02x5               g132(.a(new_n226), .b(new_n224), .c(new_n222), .d(new_n225), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g134(.clk(\a[21] ), .clkout(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(\a[22] ), .clkout(new_n231));
  xroi22aa1d04x5               g136(.a(new_n230), .b(\b[20] ), .c(new_n231), .d(\b[21] ), .out0(new_n232));
  nanp03aa1n02x5               g137(.a(new_n232), .b(new_n197), .c(new_n214), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(\b[21] ), .clkout(new_n234));
  oaoi03aa1n02x5               g139(.a(new_n231), .b(new_n234), .c(new_n224), .o1(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n235), .clkout(new_n236));
  aoi012aa1n02x5               g141(.a(new_n236), .b(new_n220), .c(new_n232), .o1(new_n237));
  aoai13aa1n02x5               g142(.a(new_n237), .b(new_n233), .c(new_n183), .d(new_n189), .o1(new_n238));
  xorb03aa1n02x5               g143(.a(new_n238), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  xorc02aa1n02x5               g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  xorc02aa1n02x5               g146(.a(\a[24] ), .b(\b[23] ), .out0(new_n242));
  aoi112aa1n02x5               g147(.a(new_n240), .b(new_n242), .c(new_n238), .d(new_n241), .o1(new_n243));
  aoai13aa1n02x5               g148(.a(new_n242), .b(new_n240), .c(new_n238), .d(new_n241), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n244), .b(new_n243), .out0(\s[24] ));
  and002aa1n02x5               g150(.a(new_n242), .b(new_n241), .o(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n246), .clkout(new_n247));
  nano32aa1n02x4               g152(.a(new_n247), .b(new_n232), .c(new_n197), .d(new_n214), .out0(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n219), .clkout(new_n249));
  aoai13aa1n02x5               g154(.a(new_n232), .b(new_n249), .c(new_n214), .d(new_n199), .o1(new_n250));
  orn002aa1n02x5               g155(.a(\a[23] ), .b(\b[22] ), .o(new_n251));
  oao003aa1n02x5               g156(.a(\a[24] ), .b(\b[23] ), .c(new_n251), .carry(new_n252));
  aoai13aa1n02x5               g157(.a(new_n252), .b(new_n247), .c(new_n250), .d(new_n235), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n253), .c(new_n190), .d(new_n248), .o1(new_n255));
  aoi112aa1n02x5               g160(.a(new_n254), .b(new_n253), .c(new_n190), .d(new_n248), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n255), .b(new_n256), .out0(\s[25] ));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  xorc02aa1n02x5               g163(.a(\a[26] ), .b(\b[25] ), .out0(new_n259));
  nona22aa1n02x4               g164(.a(new_n255), .b(new_n259), .c(new_n258), .out0(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n258), .clkout(new_n261));
  aobi12aa1n02x5               g166(.a(new_n259), .b(new_n255), .c(new_n261), .out0(new_n262));
  norb02aa1n02x5               g167(.a(new_n260), .b(new_n262), .out0(\s[26] ));
  nanp02aa1n02x5               g168(.a(new_n156), .b(new_n184), .o1(new_n264));
  nona22aa1n02x4               g169(.a(new_n264), .b(new_n188), .c(new_n185), .out0(new_n265));
  and002aa1n02x5               g170(.a(new_n259), .b(new_n254), .o(new_n266));
  nano22aa1n02x4               g171(.a(new_n233), .b(new_n246), .c(new_n266), .out0(new_n267));
  aoai13aa1n02x5               g172(.a(new_n267), .b(new_n265), .c(new_n124), .d(new_n182), .o1(new_n268));
  oao003aa1n02x5               g173(.a(\a[26] ), .b(\b[25] ), .c(new_n261), .carry(new_n269));
  aobi12aa1n02x5               g174(.a(new_n269), .b(new_n253), .c(new_n266), .out0(new_n270));
  xorc02aa1n02x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  xnbna2aa1n03x5               g176(.a(new_n271), .b(new_n270), .c(new_n268), .out0(\s[27] ));
  norp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n273), .clkout(new_n274));
  aobi12aa1n02x5               g179(.a(new_n271), .b(new_n270), .c(new_n268), .out0(new_n275));
  xnrc02aa1n02x5               g180(.a(\b[27] ), .b(\a[28] ), .out0(new_n276));
  nano22aa1n02x4               g181(.a(new_n275), .b(new_n274), .c(new_n276), .out0(new_n277));
  aobi12aa1n02x5               g182(.a(new_n267), .b(new_n183), .c(new_n189), .out0(new_n278));
  aoai13aa1n02x5               g183(.a(new_n246), .b(new_n236), .c(new_n220), .d(new_n232), .o1(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n266), .clkout(new_n280));
  aoai13aa1n02x5               g185(.a(new_n269), .b(new_n280), .c(new_n279), .d(new_n252), .o1(new_n281));
  oai012aa1n02x5               g186(.a(new_n271), .b(new_n281), .c(new_n278), .o1(new_n282));
  aoi012aa1n02x5               g187(.a(new_n276), .b(new_n282), .c(new_n274), .o1(new_n283));
  norp02aa1n02x5               g188(.a(new_n283), .b(new_n277), .o1(\s[28] ));
  norb02aa1n02x5               g189(.a(new_n271), .b(new_n276), .out0(new_n285));
  aobi12aa1n02x5               g190(.a(new_n285), .b(new_n270), .c(new_n268), .out0(new_n286));
  oao003aa1n02x5               g191(.a(\a[28] ), .b(\b[27] ), .c(new_n274), .carry(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  nano22aa1n02x4               g193(.a(new_n286), .b(new_n287), .c(new_n288), .out0(new_n289));
  oai012aa1n02x5               g194(.a(new_n285), .b(new_n281), .c(new_n278), .o1(new_n290));
  aoi012aa1n02x5               g195(.a(new_n288), .b(new_n290), .c(new_n287), .o1(new_n291));
  norp02aa1n02x5               g196(.a(new_n291), .b(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n114), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g198(.a(new_n271), .b(new_n288), .c(new_n276), .out0(new_n294));
  aobi12aa1n02x5               g199(.a(new_n294), .b(new_n270), .c(new_n268), .out0(new_n295));
  oao003aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n287), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .out0(new_n297));
  nano22aa1n02x4               g202(.a(new_n295), .b(new_n296), .c(new_n297), .out0(new_n298));
  oai012aa1n02x5               g203(.a(new_n294), .b(new_n281), .c(new_n278), .o1(new_n299));
  aoi012aa1n02x5               g204(.a(new_n297), .b(new_n299), .c(new_n296), .o1(new_n300));
  norp02aa1n02x5               g205(.a(new_n300), .b(new_n298), .o1(\s[30] ));
  norb02aa1n02x5               g206(.a(new_n294), .b(new_n297), .out0(new_n302));
  aobi12aa1n02x5               g207(.a(new_n302), .b(new_n270), .c(new_n268), .out0(new_n303));
  oao003aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  nano22aa1n02x4               g210(.a(new_n303), .b(new_n304), .c(new_n305), .out0(new_n306));
  oai012aa1n02x5               g211(.a(new_n302), .b(new_n281), .c(new_n278), .o1(new_n307));
  aoi012aa1n02x5               g212(.a(new_n305), .b(new_n307), .c(new_n304), .o1(new_n308));
  norp02aa1n02x5               g213(.a(new_n308), .b(new_n306), .o1(\s[31] ));
  xnrb03aa1n02x5               g214(.a(new_n116), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g215(.a(\a[3] ), .b(\b[2] ), .c(new_n116), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g217(.a(new_n118), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  160nm_ficinv00aa1n08x5       g218(.clk(new_n118), .clkout(new_n314));
  oaoi03aa1n02x5               g219(.a(\a[5] ), .b(\b[4] ), .c(new_n314), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oai013aa1n02x4               g221(.a(new_n121), .b(new_n314), .c(new_n105), .d(new_n106), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g223(.a(new_n102), .b(new_n317), .c(new_n103), .o1(new_n319));
  xnrb03aa1n02x5               g224(.a(new_n319), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g225(.a(new_n148), .b(new_n119), .c(new_n123), .out0(\s[9] ));
endmodule


