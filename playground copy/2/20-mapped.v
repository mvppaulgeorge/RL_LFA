// Benchmark "adder" written by ABC on Wed Jul 10 17:24:11 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n321, new_n323, new_n324, new_n326, new_n327;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\a[2] ), .clkout(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\b[1] ), .clkout(new_n100));
  nanp02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  oaoi03aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
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
  orn002aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .o(new_n118));
  oaoi03aa1n02x5               g023(.a(\a[6] ), .b(\b[5] ), .c(new_n118), .o1(new_n119));
  oai012aa1n02x5               g024(.a(new_n111), .b(new_n112), .c(new_n110), .o1(new_n120));
  oaib12aa1n02x5               g025(.a(new_n120), .b(new_n114), .c(new_n119), .out0(new_n121));
  xorc02aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .out0(new_n122));
  aoai13aa1n02x5               g027(.a(new_n122), .b(new_n121), .c(new_n109), .d(new_n117), .o1(new_n123));
  norp02aa1n02x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n124), .out0(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n123), .c(new_n98), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g032(.clk(new_n126), .clkout(new_n128));
  aoi012aa1n02x5               g033(.a(new_n124), .b(new_n97), .c(new_n125), .o1(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n128), .c(new_n123), .d(new_n98), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norp02aa1n02x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  aoi112aa1n02x5               g041(.a(new_n136), .b(new_n132), .c(new_n130), .d(new_n133), .o1(new_n137));
  aoai13aa1n02x5               g042(.a(new_n136), .b(new_n132), .c(new_n130), .d(new_n133), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(\s[12] ));
  nanp02aa1n02x5               g044(.a(new_n109), .b(new_n117), .o1(new_n140));
  nano23aa1n02x4               g045(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n141));
  aobi12aa1n02x5               g046(.a(new_n120), .b(new_n141), .c(new_n119), .out0(new_n142));
  nano23aa1n02x4               g047(.a(new_n132), .b(new_n134), .c(new_n135), .d(new_n133), .out0(new_n143));
  nanp03aa1n02x5               g048(.a(new_n143), .b(new_n122), .c(new_n126), .o1(new_n144));
  nona23aa1n02x4               g049(.a(new_n135), .b(new_n133), .c(new_n132), .d(new_n134), .out0(new_n145));
  oa0012aa1n02x5               g050(.a(new_n135), .b(new_n134), .c(new_n132), .o(new_n146));
  160nm_ficinv00aa1n08x5       g051(.clk(new_n146), .clkout(new_n147));
  oai012aa1n02x5               g052(.a(new_n147), .b(new_n145), .c(new_n129), .o1(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(new_n148), .clkout(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n144), .c(new_n140), .d(new_n142), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n152), .b(new_n150), .c(new_n153), .o1(new_n154));
  xnrb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g060(.a(\b[14] ), .b(\a[15] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  nanb02aa1n02x5               g062(.a(new_n156), .b(new_n157), .out0(new_n158));
  160nm_ficinv00aa1n08x5       g063(.clk(new_n158), .clkout(new_n159));
  norp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nano23aa1n02x4               g066(.a(new_n152), .b(new_n160), .c(new_n161), .d(new_n153), .out0(new_n162));
  aoi012aa1n02x5               g067(.a(new_n160), .b(new_n152), .c(new_n161), .o1(new_n163));
  160nm_ficinv00aa1n08x5       g068(.clk(new_n163), .clkout(new_n164));
  aoai13aa1n02x5               g069(.a(new_n159), .b(new_n164), .c(new_n150), .d(new_n162), .o1(new_n165));
  aoi112aa1n02x5               g070(.a(new_n159), .b(new_n164), .c(new_n150), .d(new_n162), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n165), .b(new_n166), .out0(\s[15] ));
  xorc02aa1n02x5               g072(.a(\a[16] ), .b(\b[15] ), .out0(new_n168));
  nona22aa1n02x4               g073(.a(new_n165), .b(new_n168), .c(new_n156), .out0(new_n169));
  xnrc02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .out0(new_n170));
  oaoi13aa1n02x5               g075(.a(new_n170), .b(new_n165), .c(\a[15] ), .d(\b[14] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n169), .b(new_n171), .out0(\s[16] ));
  nanp03aa1n02x5               g077(.a(new_n162), .b(new_n159), .c(new_n168), .o1(new_n173));
  norp02aa1n02x5               g078(.a(new_n173), .b(new_n144), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n121), .c(new_n109), .d(new_n117), .o1(new_n175));
  norp02aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  160nm_ficinv00aa1n08x5       g081(.clk(new_n176), .clkout(new_n177));
  nona22aa1n02x4               g082(.a(new_n168), .b(new_n163), .c(new_n158), .out0(new_n178));
  aoi112aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(new_n179), .clkout(new_n180));
  160nm_ficinv00aa1n08x5       g085(.clk(new_n129), .clkout(new_n181));
  nanp02aa1n02x5               g086(.a(new_n143), .b(new_n181), .o1(new_n182));
  aoi012aa1n02x5               g087(.a(new_n173), .b(new_n182), .c(new_n147), .o1(new_n183));
  nano32aa1n02x4               g088(.a(new_n183), .b(new_n178), .c(new_n180), .d(new_n177), .out0(new_n184));
  nanp02aa1n02x5               g089(.a(new_n175), .b(new_n184), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[18] ), .clkout(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(\a[17] ), .clkout(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(\b[16] ), .clkout(new_n189));
  oaoi03aa1n02x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  xroi22aa1d04x5               g096(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(new_n192), .clkout(new_n193));
  norp02aa1n02x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  aoi112aa1n02x5               g099(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n195));
  norp02aa1n02x5               g100(.a(new_n195), .b(new_n194), .o1(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n193), .c(new_n175), .d(new_n184), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  norp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanp02aa1n02x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  aoi112aa1n02x5               g110(.a(new_n200), .b(new_n205), .c(new_n197), .d(new_n202), .o1(new_n206));
  aoai13aa1n02x5               g111(.a(new_n205), .b(new_n200), .c(new_n197), .d(new_n201), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(\s[20] ));
  nona23aa1n02x4               g113(.a(new_n204), .b(new_n201), .c(new_n200), .d(new_n203), .out0(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n209), .clkout(new_n210));
  nanp02aa1n02x5               g115(.a(new_n192), .b(new_n210), .o1(new_n211));
  oai012aa1n02x5               g116(.a(new_n204), .b(new_n203), .c(new_n200), .o1(new_n212));
  oai012aa1n02x5               g117(.a(new_n212), .b(new_n209), .c(new_n196), .o1(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n213), .clkout(new_n214));
  aoai13aa1n02x5               g119(.a(new_n214), .b(new_n211), .c(new_n175), .d(new_n184), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  xorc02aa1n02x5               g123(.a(\a[22] ), .b(\b[21] ), .out0(new_n219));
  aoi112aa1n02x5               g124(.a(new_n217), .b(new_n219), .c(new_n215), .d(new_n218), .o1(new_n220));
  aoai13aa1n02x5               g125(.a(new_n219), .b(new_n217), .c(new_n215), .d(new_n218), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g127(.clk(\a[21] ), .clkout(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(\a[22] ), .clkout(new_n224));
  xroi22aa1d04x5               g129(.a(new_n223), .b(\b[20] ), .c(new_n224), .d(\b[21] ), .out0(new_n225));
  nanp03aa1n02x5               g130(.a(new_n225), .b(new_n192), .c(new_n210), .o1(new_n226));
  oai112aa1n02x5               g131(.a(new_n202), .b(new_n205), .c(new_n195), .d(new_n194), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n225), .clkout(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(\b[21] ), .clkout(new_n229));
  oaoi03aa1n02x5               g134(.a(new_n224), .b(new_n229), .c(new_n217), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n228), .c(new_n227), .d(new_n212), .o1(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(new_n231), .clkout(new_n232));
  aoai13aa1n02x5               g137(.a(new_n232), .b(new_n226), .c(new_n175), .d(new_n184), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  xorc02aa1n02x5               g140(.a(\a[23] ), .b(\b[22] ), .out0(new_n236));
  xorc02aa1n02x5               g141(.a(\a[24] ), .b(\b[23] ), .out0(new_n237));
  aoi112aa1n02x5               g142(.a(new_n235), .b(new_n237), .c(new_n233), .d(new_n236), .o1(new_n238));
  aoai13aa1n02x5               g143(.a(new_n237), .b(new_n235), .c(new_n233), .d(new_n236), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(\s[24] ));
  and002aa1n02x5               g145(.a(new_n237), .b(new_n236), .o(new_n241));
  nanb03aa1n02x5               g146(.a(new_n211), .b(new_n241), .c(new_n225), .out0(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(\a[24] ), .clkout(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(\b[23] ), .clkout(new_n244));
  oao003aa1n02x5               g149(.a(new_n243), .b(new_n244), .c(new_n235), .carry(new_n245));
  aoi012aa1n02x5               g150(.a(new_n245), .b(new_n231), .c(new_n241), .o1(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n242), .c(new_n175), .d(new_n184), .o1(new_n247));
  xorb03aa1n02x5               g152(.a(new_n247), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g153(.a(\b[24] ), .b(\a[25] ), .o1(new_n249));
  xorc02aa1n02x5               g154(.a(\a[25] ), .b(\b[24] ), .out0(new_n250));
  xorc02aa1n02x5               g155(.a(\a[26] ), .b(\b[25] ), .out0(new_n251));
  aoi112aa1n02x5               g156(.a(new_n249), .b(new_n251), .c(new_n247), .d(new_n250), .o1(new_n252));
  aoai13aa1n02x5               g157(.a(new_n251), .b(new_n249), .c(new_n247), .d(new_n250), .o1(new_n253));
  norb02aa1n02x5               g158(.a(new_n253), .b(new_n252), .out0(\s[26] ));
  oao003aa1n02x5               g159(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n255));
  nano23aa1n02x4               g160(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n256));
  aobi12aa1n02x5               g161(.a(new_n108), .b(new_n256), .c(new_n255), .out0(new_n257));
  nona22aa1n02x4               g162(.a(new_n141), .b(new_n115), .c(new_n116), .out0(new_n258));
  oai012aa1n02x5               g163(.a(new_n142), .b(new_n257), .c(new_n258), .o1(new_n259));
  norp03aa1n02x5               g164(.a(new_n170), .b(new_n163), .c(new_n158), .o1(new_n260));
  nona23aa1n02x4               g165(.a(new_n161), .b(new_n153), .c(new_n152), .d(new_n160), .out0(new_n261));
  norp03aa1n02x5               g166(.a(new_n261), .b(new_n170), .c(new_n158), .o1(new_n262));
  nanp02aa1n02x5               g167(.a(new_n148), .b(new_n262), .o1(new_n263));
  nona32aa1n02x4               g168(.a(new_n263), .b(new_n179), .c(new_n260), .d(new_n176), .out0(new_n264));
  and002aa1n02x5               g169(.a(new_n251), .b(new_n250), .o(new_n265));
  nano22aa1n02x4               g170(.a(new_n226), .b(new_n241), .c(new_n265), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n266), .b(new_n264), .c(new_n259), .d(new_n174), .o1(new_n267));
  aoai13aa1n02x5               g172(.a(new_n265), .b(new_n245), .c(new_n231), .d(new_n241), .o1(new_n268));
  oai022aa1n02x5               g173(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n269));
  aob012aa1n02x5               g174(.a(new_n269), .b(\b[25] ), .c(\a[26] ), .out0(new_n270));
  xorc02aa1n02x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n271), .clkout(new_n272));
  aoi013aa1n02x4               g177(.a(new_n272), .b(new_n267), .c(new_n268), .d(new_n270), .o1(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n230), .clkout(new_n274));
  aoai13aa1n02x5               g179(.a(new_n241), .b(new_n274), .c(new_n213), .d(new_n225), .o1(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n245), .clkout(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n265), .clkout(new_n277));
  aoai13aa1n02x5               g182(.a(new_n270), .b(new_n277), .c(new_n275), .d(new_n276), .o1(new_n278));
  aoi112aa1n02x5               g183(.a(new_n278), .b(new_n271), .c(new_n185), .d(new_n266), .o1(new_n279));
  norp02aa1n02x5               g184(.a(new_n273), .b(new_n279), .o1(\s[27] ));
  norp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n281), .clkout(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .out0(new_n283));
  nano22aa1n02x4               g188(.a(new_n273), .b(new_n282), .c(new_n283), .out0(new_n284));
  aobi12aa1n02x5               g189(.a(new_n266), .b(new_n175), .c(new_n184), .out0(new_n285));
  oai012aa1n02x5               g190(.a(new_n271), .b(new_n278), .c(new_n285), .o1(new_n286));
  aoi012aa1n02x5               g191(.a(new_n283), .b(new_n286), .c(new_n282), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n287), .b(new_n284), .o1(\s[28] ));
  norb02aa1n02x5               g193(.a(new_n271), .b(new_n283), .out0(new_n289));
  oai012aa1n02x5               g194(.a(new_n289), .b(new_n278), .c(new_n285), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[28] ), .b(\a[29] ), .out0(new_n292));
  aoi012aa1n02x5               g197(.a(new_n292), .b(new_n290), .c(new_n291), .o1(new_n293));
  160nm_ficinv00aa1n08x5       g198(.clk(new_n289), .clkout(new_n294));
  aoi013aa1n02x4               g199(.a(new_n294), .b(new_n267), .c(new_n268), .d(new_n270), .o1(new_n295));
  nano22aa1n02x4               g200(.a(new_n295), .b(new_n291), .c(new_n292), .out0(new_n296));
  norp02aa1n02x5               g201(.a(new_n293), .b(new_n296), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g203(.a(new_n271), .b(new_n292), .c(new_n283), .out0(new_n299));
  oai012aa1n02x5               g204(.a(new_n299), .b(new_n278), .c(new_n285), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[29] ), .b(\a[30] ), .out0(new_n302));
  aoi012aa1n02x5               g207(.a(new_n302), .b(new_n300), .c(new_n301), .o1(new_n303));
  160nm_ficinv00aa1n08x5       g208(.clk(new_n299), .clkout(new_n304));
  aoi013aa1n02x4               g209(.a(new_n304), .b(new_n267), .c(new_n268), .d(new_n270), .o1(new_n305));
  nano22aa1n02x4               g210(.a(new_n305), .b(new_n301), .c(new_n302), .out0(new_n306));
  norp02aa1n02x5               g211(.a(new_n303), .b(new_n306), .o1(\s[30] ));
  norb02aa1n02x5               g212(.a(new_n299), .b(new_n302), .out0(new_n308));
  160nm_ficinv00aa1n08x5       g213(.clk(new_n308), .clkout(new_n309));
  aoi013aa1n02x4               g214(.a(new_n309), .b(new_n267), .c(new_n268), .d(new_n270), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[30] ), .b(\a[31] ), .out0(new_n312));
  nano22aa1n02x4               g217(.a(new_n310), .b(new_n311), .c(new_n312), .out0(new_n313));
  oai012aa1n02x5               g218(.a(new_n308), .b(new_n278), .c(new_n285), .o1(new_n314));
  aoi012aa1n02x5               g219(.a(new_n312), .b(new_n314), .c(new_n311), .o1(new_n315));
  norp02aa1n02x5               g220(.a(new_n315), .b(new_n313), .o1(\s[31] ));
  xnrb03aa1n02x5               g221(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g222(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g224(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g225(.a(\a[5] ), .b(\b[4] ), .c(new_n257), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g227(.a(\a[5] ), .b(\b[4] ), .c(new_n257), .carry(new_n323));
  oaoi03aa1n02x5               g228(.a(\a[6] ), .b(\b[5] ), .c(new_n323), .o1(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  160nm_ficinv00aa1n08x5       g230(.clk(\a[8] ), .clkout(new_n326));
  aoi012aa1n02x5               g231(.a(new_n112), .b(new_n324), .c(new_n113), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[7] ), .c(new_n326), .out0(\s[8] ));
  xnbna2aa1n03x5               g233(.a(new_n122), .b(new_n140), .c(new_n142), .out0(\s[9] ));
endmodule


