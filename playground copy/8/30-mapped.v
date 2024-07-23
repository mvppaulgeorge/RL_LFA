// Benchmark "adder" written by ABC on Thu Jul 11 11:30:24 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n316, new_n317, new_n319, new_n321,
    new_n322, new_n324;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n02x4               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  oai012aa1n02x5               g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  norp03aa1n02x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\a[6] ), .clkout(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\b[5] ), .clkout(new_n119));
  aoi112aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n120));
  aoi012aa1n02x5               g025(.a(new_n120), .b(new_n118), .c(new_n119), .o1(new_n121));
  aoi012aa1n02x5               g026(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n122));
  oai012aa1n02x5               g027(.a(new_n122), .b(new_n114), .c(new_n121), .o1(new_n123));
  nanp02aa1n02x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n124), .b(new_n97), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n126));
  norp02aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  nanp02aa1n02x5               g035(.a(new_n126), .b(new_n98), .o1(new_n131));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  aoi012aa1n02x5               g039(.a(new_n127), .b(new_n97), .c(new_n128), .o1(new_n135));
  160nm_ficinv00aa1n08x5       g040(.clk(new_n135), .clkout(new_n136));
  aoai13aa1n02x5               g041(.a(new_n134), .b(new_n136), .c(new_n131), .d(new_n129), .o1(new_n137));
  aoi112aa1n02x5               g042(.a(new_n136), .b(new_n134), .c(new_n131), .d(new_n129), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(\s[11] ));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  nona22aa1n02x4               g047(.a(new_n137), .b(new_n142), .c(new_n132), .out0(new_n143));
  160nm_ficinv00aa1n08x5       g048(.clk(new_n142), .clkout(new_n144));
  oaoi13aa1n02x5               g049(.a(new_n144), .b(new_n137), .c(\a[11] ), .d(\b[10] ), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n143), .b(new_n145), .out0(\s[12] ));
  nano23aa1n02x4               g051(.a(new_n132), .b(new_n140), .c(new_n141), .d(new_n133), .out0(new_n147));
  nano23aa1n02x4               g052(.a(new_n97), .b(new_n127), .c(new_n128), .d(new_n124), .out0(new_n148));
  nanp02aa1n02x5               g053(.a(new_n148), .b(new_n147), .o1(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n149), .clkout(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n151));
  aoi112aa1n02x5               g056(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n152));
  aoi112aa1n02x5               g057(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n153));
  oai112aa1n02x5               g058(.a(new_n142), .b(new_n134), .c(new_n153), .d(new_n127), .o1(new_n154));
  nona22aa1n02x4               g059(.a(new_n154), .b(new_n152), .c(new_n140), .out0(new_n155));
  160nm_ficinv00aa1n08x5       g060(.clk(new_n155), .clkout(new_n156));
  norp02aa1n02x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  xnbna2aa1n03x5               g064(.a(new_n159), .b(new_n151), .c(new_n156), .out0(\s[13] ));
  nanp02aa1n02x5               g065(.a(new_n151), .b(new_n156), .o1(new_n161));
  aoi012aa1n02x5               g066(.a(new_n157), .b(new_n161), .c(new_n158), .o1(new_n162));
  xnrb03aa1n02x5               g067(.a(new_n162), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nona23aa1n02x4               g070(.a(new_n165), .b(new_n158), .c(new_n157), .d(new_n164), .out0(new_n166));
  aoi012aa1n02x5               g071(.a(new_n164), .b(new_n157), .c(new_n165), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n166), .c(new_n151), .d(new_n156), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  norp02aa1n02x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nanp02aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  aoi112aa1n02x5               g079(.a(new_n174), .b(new_n170), .c(new_n168), .d(new_n171), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n174), .b(new_n170), .c(new_n168), .d(new_n171), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(\s[16] ));
  norb02aa1n02x5               g082(.a(new_n165), .b(new_n164), .out0(new_n178));
  nano23aa1n02x4               g083(.a(new_n170), .b(new_n172), .c(new_n173), .d(new_n171), .out0(new_n179));
  nano32aa1n02x4               g084(.a(new_n149), .b(new_n179), .c(new_n159), .d(new_n178), .out0(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n181));
  nona23aa1n02x4               g086(.a(new_n173), .b(new_n171), .c(new_n170), .d(new_n172), .out0(new_n182));
  norp02aa1n02x5               g087(.a(new_n182), .b(new_n166), .o1(new_n183));
  aoi112aa1n02x5               g088(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n184));
  oai022aa1n02x5               g089(.a(new_n182), .b(new_n167), .c(\b[15] ), .d(\a[16] ), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n185), .b(new_n184), .c(new_n155), .d(new_n183), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(new_n186), .b(new_n181), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g093(.clk(\a[18] ), .clkout(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(\a[17] ), .clkout(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(\b[16] ), .clkout(new_n191));
  oaoi03aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  xroi22aa1d04x5               g098(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n194));
  nanp02aa1n02x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  nona22aa1n02x4               g100(.a(new_n195), .b(\b[16] ), .c(\a[17] ), .out0(new_n196));
  oaib12aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(new_n189), .out0(new_n197));
  norp02aa1n02x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  aoai13aa1n02x5               g105(.a(new_n200), .b(new_n197), .c(new_n187), .d(new_n194), .o1(new_n201));
  aoi112aa1n02x5               g106(.a(new_n200), .b(new_n197), .c(new_n187), .d(new_n194), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n201), .b(new_n202), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  nona22aa1n02x4               g112(.a(new_n201), .b(new_n207), .c(new_n198), .out0(new_n208));
  orn002aa1n02x5               g113(.a(\a[19] ), .b(\b[18] ), .o(new_n209));
  aobi12aa1n02x5               g114(.a(new_n207), .b(new_n201), .c(new_n209), .out0(new_n210));
  norb02aa1n02x5               g115(.a(new_n208), .b(new_n210), .out0(\s[20] ));
  nano23aa1n02x4               g116(.a(new_n198), .b(new_n205), .c(new_n206), .d(new_n199), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(new_n194), .b(new_n212), .o1(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n213), .clkout(new_n214));
  norp02aa1n02x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  aoi013aa1n02x4               g120(.a(new_n215), .b(new_n195), .c(new_n190), .d(new_n191), .o1(new_n216));
  nona23aa1n02x4               g121(.a(new_n206), .b(new_n199), .c(new_n198), .d(new_n205), .out0(new_n217));
  oaoi03aa1n02x5               g122(.a(\a[20] ), .b(\b[19] ), .c(new_n209), .o1(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n218), .clkout(new_n219));
  oai012aa1n02x5               g124(.a(new_n219), .b(new_n217), .c(new_n216), .o1(new_n220));
  xorc02aa1n02x5               g125(.a(\a[21] ), .b(\b[20] ), .out0(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n220), .c(new_n187), .d(new_n214), .o1(new_n222));
  aoi112aa1n02x5               g127(.a(new_n221), .b(new_n220), .c(new_n187), .d(new_n214), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n222), .b(new_n223), .out0(\s[21] ));
  xnrc02aa1n02x5               g129(.a(\b[21] ), .b(\a[22] ), .out0(new_n225));
  oai112aa1n02x5               g130(.a(new_n222), .b(new_n225), .c(\b[20] ), .d(\a[21] ), .o1(new_n226));
  oaoi13aa1n02x5               g131(.a(new_n225), .b(new_n222), .c(\a[21] ), .d(\b[20] ), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n226), .b(new_n227), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g133(.clk(\a[21] ), .clkout(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(\a[22] ), .clkout(new_n230));
  xroi22aa1d04x5               g135(.a(new_n229), .b(\b[20] ), .c(new_n230), .d(\b[21] ), .out0(new_n231));
  and003aa1n02x5               g136(.a(new_n194), .b(new_n231), .c(new_n212), .o(new_n232));
  aoai13aa1n02x5               g137(.a(new_n231), .b(new_n218), .c(new_n212), .d(new_n197), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(\b[21] ), .clkout(new_n234));
  norp02aa1n02x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  oao003aa1n02x5               g140(.a(new_n230), .b(new_n234), .c(new_n235), .carry(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n236), .clkout(new_n237));
  nanp02aa1n02x5               g142(.a(new_n233), .b(new_n237), .o1(new_n238));
  xorc02aa1n02x5               g143(.a(\a[23] ), .b(\b[22] ), .out0(new_n239));
  aoai13aa1n02x5               g144(.a(new_n239), .b(new_n238), .c(new_n187), .d(new_n232), .o1(new_n240));
  aoi112aa1n02x5               g145(.a(new_n239), .b(new_n238), .c(new_n187), .d(new_n232), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n240), .b(new_n241), .out0(\s[23] ));
  norp02aa1n02x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  xorc02aa1n02x5               g148(.a(\a[24] ), .b(\b[23] ), .out0(new_n244));
  nona22aa1n02x4               g149(.a(new_n240), .b(new_n244), .c(new_n243), .out0(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n244), .clkout(new_n246));
  oaoi13aa1n02x5               g151(.a(new_n246), .b(new_n240), .c(\a[23] ), .d(\b[22] ), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n245), .b(new_n247), .out0(\s[24] ));
  160nm_ficinv00aa1n08x5       g153(.clk(\a[23] ), .clkout(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(\a[24] ), .clkout(new_n250));
  xroi22aa1d04x5               g155(.a(new_n249), .b(\b[22] ), .c(new_n250), .d(\b[23] ), .out0(new_n251));
  nanb03aa1n02x5               g156(.a(new_n213), .b(new_n251), .c(new_n231), .out0(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n251), .clkout(new_n253));
  oai022aa1n02x5               g158(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n254));
  oaib12aa1n02x5               g159(.a(new_n254), .b(new_n250), .c(\b[23] ), .out0(new_n255));
  aoai13aa1n02x5               g160(.a(new_n255), .b(new_n253), .c(new_n233), .d(new_n237), .o1(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n256), .clkout(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n252), .c(new_n181), .d(new_n186), .o1(new_n258));
  xorb03aa1n02x5               g163(.a(new_n258), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  xorc02aa1n02x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  norp02aa1n02x5               g166(.a(\b[25] ), .b(\a[26] ), .o1(new_n262));
  nanp02aa1n02x5               g167(.a(\b[25] ), .b(\a[26] ), .o1(new_n263));
  norb02aa1n02x5               g168(.a(new_n263), .b(new_n262), .out0(new_n264));
  aoi112aa1n02x5               g169(.a(new_n260), .b(new_n264), .c(new_n258), .d(new_n261), .o1(new_n265));
  aoai13aa1n02x5               g170(.a(new_n264), .b(new_n260), .c(new_n258), .d(new_n261), .o1(new_n266));
  norb02aa1n02x5               g171(.a(new_n266), .b(new_n265), .out0(\s[26] ));
  nanp02aa1n02x5               g172(.a(new_n261), .b(new_n264), .o1(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n268), .clkout(new_n269));
  nano32aa1n02x4               g174(.a(new_n213), .b(new_n269), .c(new_n231), .d(new_n251), .out0(new_n270));
  nanp02aa1n02x5               g175(.a(new_n187), .b(new_n270), .o1(new_n271));
  oai012aa1n02x5               g176(.a(new_n263), .b(new_n262), .c(new_n260), .o1(new_n272));
  aobi12aa1n02x5               g177(.a(new_n272), .b(new_n256), .c(new_n269), .out0(new_n273));
  xorc02aa1n02x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n274), .b(new_n273), .c(new_n271), .out0(\s[27] ));
  norp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n276), .clkout(new_n277));
  aobi12aa1n02x5               g182(.a(new_n274), .b(new_n273), .c(new_n271), .out0(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[27] ), .b(\a[28] ), .out0(new_n279));
  nano22aa1n02x4               g184(.a(new_n278), .b(new_n277), .c(new_n279), .out0(new_n280));
  aobi12aa1n02x5               g185(.a(new_n270), .b(new_n186), .c(new_n181), .out0(new_n281));
  aoai13aa1n02x5               g186(.a(new_n251), .b(new_n236), .c(new_n220), .d(new_n231), .o1(new_n282));
  aoai13aa1n02x5               g187(.a(new_n272), .b(new_n268), .c(new_n282), .d(new_n255), .o1(new_n283));
  oai012aa1n02x5               g188(.a(new_n274), .b(new_n283), .c(new_n281), .o1(new_n284));
  aoi012aa1n02x5               g189(.a(new_n279), .b(new_n284), .c(new_n277), .o1(new_n285));
  norp02aa1n02x5               g190(.a(new_n285), .b(new_n280), .o1(\s[28] ));
  norb02aa1n02x5               g191(.a(new_n274), .b(new_n279), .out0(new_n287));
  aobi12aa1n02x5               g192(.a(new_n287), .b(new_n273), .c(new_n271), .out0(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n277), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n288), .b(new_n289), .c(new_n290), .out0(new_n291));
  oai012aa1n02x5               g196(.a(new_n287), .b(new_n283), .c(new_n281), .o1(new_n292));
  aoi012aa1n02x5               g197(.a(new_n290), .b(new_n292), .c(new_n289), .o1(new_n293));
  norp02aa1n02x5               g198(.a(new_n293), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g200(.a(new_n274), .b(new_n290), .c(new_n279), .out0(new_n296));
  aobi12aa1n02x5               g201(.a(new_n296), .b(new_n273), .c(new_n271), .out0(new_n297));
  oao003aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[29] ), .b(\a[30] ), .out0(new_n299));
  nano22aa1n02x4               g204(.a(new_n297), .b(new_n298), .c(new_n299), .out0(new_n300));
  oai012aa1n02x5               g205(.a(new_n296), .b(new_n283), .c(new_n281), .o1(new_n301));
  aoi012aa1n02x5               g206(.a(new_n299), .b(new_n301), .c(new_n298), .o1(new_n302));
  norp02aa1n02x5               g207(.a(new_n302), .b(new_n300), .o1(\s[30] ));
  norb02aa1n02x5               g208(.a(new_n296), .b(new_n299), .out0(new_n304));
  aobi12aa1n02x5               g209(.a(new_n304), .b(new_n273), .c(new_n271), .out0(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n298), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  nano22aa1n02x4               g212(.a(new_n305), .b(new_n306), .c(new_n307), .out0(new_n308));
  oai012aa1n02x5               g213(.a(new_n304), .b(new_n283), .c(new_n281), .o1(new_n309));
  aoi012aa1n02x5               g214(.a(new_n307), .b(new_n309), .c(new_n306), .o1(new_n310));
  norp02aa1n02x5               g215(.a(new_n310), .b(new_n308), .o1(\s[31] ));
  xnrb03aa1n02x5               g216(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g219(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g220(.a(new_n116), .b(new_n109), .out0(new_n316));
  oai012aa1n02x5               g221(.a(new_n316), .b(\b[4] ), .c(\a[5] ), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g223(.a(new_n118), .b(new_n119), .c(new_n317), .carry(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g225(.a(new_n118), .b(new_n119), .c(new_n317), .o1(new_n321));
  oaoi03aa1n02x5               g226(.a(\a[7] ), .b(\b[6] ), .c(new_n321), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  aoi112aa1n02x5               g228(.a(new_n125), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n324));
  norb02aa1n02x5               g229(.a(new_n126), .b(new_n324), .out0(\s[9] ));
endmodule


