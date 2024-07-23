// Benchmark "adder" written by ABC on Wed Jul 10 16:58:00 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n316, new_n317, new_n318, new_n320,
    new_n321, new_n322, new_n323, new_n325;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  and002aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(new_n100), .clkout(new_n101));
  and002aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o(new_n102));
  160nm_ficinv00aa1n08x5       g007(.clk(\a[3] ), .clkout(new_n103));
  160nm_ficinv00aa1n08x5       g008(.clk(\b[2] ), .clkout(new_n104));
  nanp02aa1n02x5               g009(.a(new_n104), .b(new_n103), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(new_n105), .b(new_n106), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[0] ), .b(\a[1] ), .o1(new_n110));
  aoi012aa1n02x5               g015(.a(new_n108), .b(new_n109), .c(new_n110), .o1(new_n111));
  oa0022aa1n02x5               g016(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n112));
  oaoi13aa1n02x5               g017(.a(new_n102), .b(new_n112), .c(new_n111), .d(new_n107), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nona23aa1n02x4               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  xnrc02aa1n02x5               g023(.a(\b[5] ), .b(\a[6] ), .out0(new_n119));
  xnrc02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .out0(new_n120));
  norp03aa1n02x5               g025(.a(new_n118), .b(new_n119), .c(new_n120), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(new_n113), .b(new_n121), .o1(new_n122));
  160nm_ficinv00aa1n08x5       g027(.clk(\a[6] ), .clkout(new_n123));
  oai022aa1n02x5               g028(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n124));
  oaib12aa1n02x5               g029(.a(new_n124), .b(new_n123), .c(\b[5] ), .out0(new_n125));
  160nm_ficinv00aa1n08x5       g030(.clk(new_n116), .clkout(new_n126));
  oaoi03aa1n02x5               g031(.a(\a[8] ), .b(\b[7] ), .c(new_n126), .o1(new_n127));
  oabi12aa1n02x5               g032(.a(new_n127), .b(new_n118), .c(new_n125), .out0(new_n128));
  xnrc02aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .out0(new_n129));
  nona22aa1n02x4               g034(.a(new_n122), .b(new_n128), .c(new_n129), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n99), .b(new_n130), .c(new_n101), .out0(\s[10] ));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  160nm_ficinv00aa1n08x5       g037(.clk(new_n132), .clkout(new_n133));
  nanp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  aoai13aa1n02x5               g039(.a(new_n98), .b(new_n97), .c(new_n130), .d(new_n101), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n133), .c(new_n134), .out0(\s[11] ));
  norb02aa1n02x5               g041(.a(new_n134), .b(new_n132), .out0(new_n137));
  160nm_ficinv00aa1n08x5       g042(.clk(new_n137), .clkout(new_n138));
  norp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanb02aa1n02x5               g045(.a(new_n139), .b(new_n140), .out0(new_n141));
  oai112aa1n02x5               g046(.a(new_n133), .b(new_n141), .c(new_n135), .d(new_n138), .o1(new_n142));
  oaoi13aa1n02x5               g047(.a(new_n141), .b(new_n133), .c(new_n135), .d(new_n138), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n142), .b(new_n143), .out0(\s[12] ));
  160nm_ficinv00aa1n08x5       g049(.clk(new_n98), .clkout(new_n145));
  nano23aa1n02x4               g050(.a(new_n132), .b(new_n139), .c(new_n140), .d(new_n134), .out0(new_n146));
  oai022aa1n02x5               g051(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n147));
  nona32aa1n02x4               g052(.a(new_n146), .b(new_n147), .c(new_n100), .d(new_n145), .out0(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(new_n148), .clkout(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n128), .c(new_n113), .d(new_n121), .o1(new_n150));
  nano32aa1n02x4               g055(.a(new_n141), .b(new_n133), .c(new_n134), .d(new_n98), .out0(new_n151));
  aoi012aa1n02x5               g056(.a(new_n139), .b(new_n132), .c(new_n140), .o1(new_n152));
  160nm_ficinv00aa1n08x5       g057(.clk(new_n152), .clkout(new_n153));
  aoi012aa1n02x5               g058(.a(new_n153), .b(new_n151), .c(new_n147), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(new_n150), .b(new_n154), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  aoi012aa1n02x5               g063(.a(new_n157), .b(new_n155), .c(new_n158), .o1(new_n159));
  xnrb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nona23aa1n02x4               g067(.a(new_n162), .b(new_n158), .c(new_n157), .d(new_n161), .out0(new_n163));
  oai012aa1n02x5               g068(.a(new_n162), .b(new_n161), .c(new_n157), .o1(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n163), .c(new_n150), .d(new_n154), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(new_n169));
  160nm_ficinv00aa1n08x5       g074(.clk(new_n169), .clkout(new_n170));
  nanp02aa1n02x5               g075(.a(new_n165), .b(new_n170), .o1(new_n171));
  norp02aa1n02x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nanp02aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(new_n174));
  oai112aa1n02x5               g079(.a(new_n171), .b(new_n174), .c(\b[14] ), .d(\a[15] ), .o1(new_n175));
  oaoi13aa1n02x5               g080(.a(new_n174), .b(new_n171), .c(\a[15] ), .d(\b[14] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n175), .b(new_n176), .out0(\s[16] ));
  nano23aa1n02x4               g082(.a(new_n157), .b(new_n161), .c(new_n162), .d(new_n158), .out0(new_n178));
  nona22aa1n02x4               g083(.a(new_n178), .b(new_n174), .c(new_n169), .out0(new_n179));
  norp02aa1n02x5               g084(.a(new_n179), .b(new_n148), .o1(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n128), .c(new_n113), .d(new_n121), .o1(new_n181));
  nona22aa1n02x4               g086(.a(new_n170), .b(new_n164), .c(new_n174), .out0(new_n182));
  norb02aa1n02x5               g087(.a(new_n140), .b(new_n139), .out0(new_n183));
  oab012aa1n02x4               g088(.a(new_n97), .b(\a[9] ), .c(\b[8] ), .out0(new_n184));
  nona23aa1n02x4               g089(.a(new_n183), .b(new_n137), .c(new_n184), .d(new_n145), .out0(new_n185));
  aoi012aa1n02x5               g090(.a(new_n179), .b(new_n185), .c(new_n152), .o1(new_n186));
  aoi012aa1n02x5               g091(.a(new_n172), .b(new_n167), .c(new_n173), .o1(new_n187));
  nano22aa1n02x4               g092(.a(new_n186), .b(new_n182), .c(new_n187), .out0(new_n188));
  nanp02aa1n02x5               g093(.a(new_n188), .b(new_n181), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g095(.clk(\a[18] ), .clkout(new_n191));
  160nm_ficinv00aa1n08x5       g096(.clk(\a[17] ), .clkout(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(\b[16] ), .clkout(new_n193));
  oaoi03aa1n02x5               g098(.a(new_n192), .b(new_n193), .c(new_n189), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[17] ), .c(new_n191), .out0(\s[18] ));
  xroi22aa1d04x5               g100(.a(new_n192), .b(\b[16] ), .c(new_n191), .d(\b[17] ), .out0(new_n196));
  160nm_ficinv00aa1n08x5       g101(.clk(new_n196), .clkout(new_n197));
  oai022aa1n02x5               g102(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n198));
  oaib12aa1n02x5               g103(.a(new_n198), .b(new_n191), .c(\b[17] ), .out0(new_n199));
  aoai13aa1n02x5               g104(.a(new_n199), .b(new_n197), .c(new_n188), .d(new_n181), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nanp02aa1n02x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  norp02aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  aoi112aa1n02x5               g112(.a(new_n203), .b(new_n207), .c(new_n200), .d(new_n204), .o1(new_n208));
  aoai13aa1n02x5               g113(.a(new_n207), .b(new_n203), .c(new_n200), .d(new_n204), .o1(new_n209));
  norb02aa1n02x5               g114(.a(new_n209), .b(new_n208), .out0(\s[20] ));
  nano23aa1n02x4               g115(.a(new_n203), .b(new_n205), .c(new_n206), .d(new_n204), .out0(new_n211));
  nanp02aa1n02x5               g116(.a(new_n196), .b(new_n211), .o1(new_n212));
  nona23aa1n02x4               g117(.a(new_n206), .b(new_n204), .c(new_n203), .d(new_n205), .out0(new_n213));
  aoi012aa1n02x5               g118(.a(new_n205), .b(new_n203), .c(new_n206), .o1(new_n214));
  oai012aa1n02x5               g119(.a(new_n214), .b(new_n213), .c(new_n199), .o1(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  aoai13aa1n02x5               g121(.a(new_n216), .b(new_n212), .c(new_n188), .d(new_n181), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  xorc02aa1n02x5               g124(.a(\a[21] ), .b(\b[20] ), .out0(new_n220));
  xorc02aa1n02x5               g125(.a(\a[22] ), .b(\b[21] ), .out0(new_n221));
  aoi112aa1n02x5               g126(.a(new_n219), .b(new_n221), .c(new_n217), .d(new_n220), .o1(new_n222));
  aoai13aa1n02x5               g127(.a(new_n221), .b(new_n219), .c(new_n217), .d(new_n220), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g129(.clk(\a[21] ), .clkout(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(\a[22] ), .clkout(new_n226));
  xroi22aa1d04x5               g131(.a(new_n225), .b(\b[20] ), .c(new_n226), .d(\b[21] ), .out0(new_n227));
  nanp03aa1n02x5               g132(.a(new_n227), .b(new_n196), .c(new_n211), .o1(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(\b[21] ), .clkout(new_n229));
  oaoi03aa1n02x5               g134(.a(new_n226), .b(new_n229), .c(new_n219), .o1(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(new_n230), .clkout(new_n231));
  aoi012aa1n02x5               g136(.a(new_n231), .b(new_n215), .c(new_n227), .o1(new_n232));
  aoai13aa1n02x5               g137(.a(new_n232), .b(new_n228), .c(new_n188), .d(new_n181), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  xorc02aa1n02x5               g140(.a(\a[23] ), .b(\b[22] ), .out0(new_n236));
  xorc02aa1n02x5               g141(.a(\a[24] ), .b(\b[23] ), .out0(new_n237));
  aoi112aa1n02x5               g142(.a(new_n235), .b(new_n237), .c(new_n233), .d(new_n236), .o1(new_n238));
  aoai13aa1n02x5               g143(.a(new_n237), .b(new_n235), .c(new_n233), .d(new_n236), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(\s[24] ));
  and002aa1n02x5               g145(.a(new_n237), .b(new_n236), .o(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n241), .clkout(new_n242));
  nano32aa1n02x4               g147(.a(new_n242), .b(new_n227), .c(new_n196), .d(new_n211), .out0(new_n243));
  nanp02aa1n02x5               g148(.a(new_n193), .b(new_n192), .o1(new_n244));
  oaoi03aa1n02x5               g149(.a(\a[18] ), .b(\b[17] ), .c(new_n244), .o1(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n214), .clkout(new_n246));
  aoai13aa1n02x5               g151(.a(new_n227), .b(new_n246), .c(new_n211), .d(new_n245), .o1(new_n247));
  norp02aa1n02x5               g152(.a(\b[23] ), .b(\a[24] ), .o1(new_n248));
  nanp02aa1n02x5               g153(.a(\b[23] ), .b(\a[24] ), .o1(new_n249));
  aoi012aa1n02x5               g154(.a(new_n248), .b(new_n235), .c(new_n249), .o1(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n242), .c(new_n247), .d(new_n230), .o1(new_n251));
  xorc02aa1n02x5               g156(.a(\a[25] ), .b(\b[24] ), .out0(new_n252));
  aoai13aa1n02x5               g157(.a(new_n252), .b(new_n251), .c(new_n189), .d(new_n243), .o1(new_n253));
  aoi112aa1n02x5               g158(.a(new_n252), .b(new_n251), .c(new_n189), .d(new_n243), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n253), .b(new_n254), .out0(\s[25] ));
  norp02aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  xorc02aa1n02x5               g161(.a(\a[26] ), .b(\b[25] ), .out0(new_n257));
  nona22aa1n02x4               g162(.a(new_n253), .b(new_n257), .c(new_n256), .out0(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n256), .clkout(new_n259));
  aobi12aa1n02x5               g164(.a(new_n257), .b(new_n253), .c(new_n259), .out0(new_n260));
  norb02aa1n02x5               g165(.a(new_n258), .b(new_n260), .out0(\s[26] ));
  160nm_ficinv00aa1n08x5       g166(.clk(new_n128), .clkout(new_n262));
  nanp02aa1n02x5               g167(.a(new_n122), .b(new_n262), .o1(new_n263));
  norp03aa1n02x5               g168(.a(new_n163), .b(new_n174), .c(new_n169), .o1(new_n264));
  aoai13aa1n02x5               g169(.a(new_n264), .b(new_n153), .c(new_n151), .d(new_n147), .o1(new_n265));
  nanp03aa1n02x5               g170(.a(new_n265), .b(new_n182), .c(new_n187), .o1(new_n266));
  and002aa1n02x5               g171(.a(new_n257), .b(new_n252), .o(new_n267));
  nano22aa1n02x4               g172(.a(new_n228), .b(new_n241), .c(new_n267), .out0(new_n268));
  aoai13aa1n02x5               g173(.a(new_n268), .b(new_n266), .c(new_n263), .d(new_n180), .o1(new_n269));
  oao003aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .c(new_n259), .carry(new_n270));
  aobi12aa1n02x5               g175(.a(new_n270), .b(new_n251), .c(new_n267), .out0(new_n271));
  norp02aa1n02x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  nanp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  norb02aa1n02x5               g178(.a(new_n273), .b(new_n272), .out0(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n274), .b(new_n271), .c(new_n269), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n272), .clkout(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  aobi12aa1n02x5               g182(.a(new_n268), .b(new_n188), .c(new_n181), .out0(new_n278));
  aoai13aa1n02x5               g183(.a(new_n241), .b(new_n231), .c(new_n215), .d(new_n227), .o1(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n267), .clkout(new_n280));
  aoai13aa1n02x5               g185(.a(new_n270), .b(new_n280), .c(new_n279), .d(new_n250), .o1(new_n281));
  oai012aa1n02x5               g186(.a(new_n273), .b(new_n281), .c(new_n278), .o1(new_n282));
  aoi012aa1n02x5               g187(.a(new_n277), .b(new_n282), .c(new_n276), .o1(new_n283));
  aobi12aa1n02x5               g188(.a(new_n273), .b(new_n271), .c(new_n269), .out0(new_n284));
  nano22aa1n02x4               g189(.a(new_n284), .b(new_n276), .c(new_n277), .out0(new_n285));
  norp02aa1n02x5               g190(.a(new_n283), .b(new_n285), .o1(\s[28] ));
  nano22aa1n02x4               g191(.a(new_n277), .b(new_n276), .c(new_n273), .out0(new_n287));
  oai012aa1n02x5               g192(.a(new_n287), .b(new_n281), .c(new_n278), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n276), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  aoi012aa1n02x5               g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  aobi12aa1n02x5               g196(.a(new_n287), .b(new_n271), .c(new_n269), .out0(new_n292));
  nano22aa1n02x4               g197(.a(new_n292), .b(new_n289), .c(new_n290), .out0(new_n293));
  norp02aa1n02x5               g198(.a(new_n291), .b(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n110), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g200(.a(new_n274), .b(new_n290), .c(new_n277), .out0(new_n296));
  oai012aa1n02x5               g201(.a(new_n296), .b(new_n281), .c(new_n278), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[29] ), .b(\a[30] ), .out0(new_n299));
  aoi012aa1n02x5               g204(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n300));
  aobi12aa1n02x5               g205(.a(new_n296), .b(new_n271), .c(new_n269), .out0(new_n301));
  nano22aa1n02x4               g206(.a(new_n301), .b(new_n298), .c(new_n299), .out0(new_n302));
  norp02aa1n02x5               g207(.a(new_n300), .b(new_n302), .o1(\s[30] ));
  norb03aa1n02x5               g208(.a(new_n287), .b(new_n299), .c(new_n290), .out0(new_n304));
  aobi12aa1n02x5               g209(.a(new_n304), .b(new_n271), .c(new_n269), .out0(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n298), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  nano22aa1n02x4               g212(.a(new_n305), .b(new_n306), .c(new_n307), .out0(new_n308));
  oai012aa1n02x5               g213(.a(new_n304), .b(new_n281), .c(new_n278), .o1(new_n309));
  aoi012aa1n02x5               g214(.a(new_n307), .b(new_n309), .c(new_n306), .o1(new_n310));
  norp02aa1n02x5               g215(.a(new_n310), .b(new_n308), .o1(\s[31] ));
  xnbna2aa1n03x5               g216(.a(new_n111), .b(new_n105), .c(new_n106), .out0(\s[3] ));
  oaoi03aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n111), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g219(.a(new_n113), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  160nm_ficinv00aa1n08x5       g220(.clk(\a[5] ), .clkout(new_n316));
  160nm_ficinv00aa1n08x5       g221(.clk(\b[4] ), .clkout(new_n317));
  oaoi03aa1n02x5               g222(.a(new_n316), .b(new_n317), .c(new_n113), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(new_n123), .out0(\s[6] ));
  and002aa1n02x5               g224(.a(\b[5] ), .b(\a[6] ), .o(new_n320));
  nanb02aa1n02x5               g225(.a(new_n119), .b(new_n318), .out0(new_n321));
  nona23aa1n02x4               g226(.a(new_n321), .b(new_n117), .c(new_n116), .d(new_n320), .out0(new_n322));
  aboi22aa1n03x5               g227(.a(new_n320), .b(new_n321), .c(new_n126), .d(new_n117), .out0(new_n323));
  norb02aa1n02x5               g228(.a(new_n322), .b(new_n323), .out0(\s[7] ));
  norb02aa1n02x5               g229(.a(new_n115), .b(new_n114), .out0(new_n325));
  xnbna2aa1n03x5               g230(.a(new_n325), .b(new_n322), .c(new_n126), .out0(\s[8] ));
  xobna2aa1n03x5               g231(.a(new_n129), .b(new_n122), .c(new_n262), .out0(\s[9] ));
endmodule


