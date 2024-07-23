// Benchmark "adder" written by ABC on Wed Jul 10 17:24:55 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n158,
    new_n159, new_n160, new_n161, new_n162, new_n163, new_n164, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n176, new_n177, new_n178, new_n179, new_n181, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n197, new_n199,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n207,
    new_n208, new_n209, new_n210, new_n211, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n223,
    new_n224, new_n225, new_n226, new_n227, new_n229, new_n230, new_n231,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n306, new_n309, new_n311, new_n313;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  norp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aoi012aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nona23aa1n02x4               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  aoi012aa1n02x5               g011(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n107));
  oai012aa1n02x5               g012(.a(new_n107), .b(new_n106), .c(new_n101), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nona23aa1n02x4               g017(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .out0(new_n115));
  norp03aa1n02x5               g020(.a(new_n113), .b(new_n114), .c(new_n115), .o1(new_n116));
  oai012aa1n02x5               g021(.a(new_n110), .b(new_n111), .c(new_n109), .o1(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\a[5] ), .clkout(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\b[4] ), .clkout(new_n119));
  nanp02aa1n02x5               g024(.a(new_n119), .b(new_n118), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n121));
  oaib12aa1n02x5               g026(.a(new_n117), .b(new_n113), .c(new_n121), .out0(new_n122));
  xorc02aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n108), .d(new_n116), .o1(new_n124));
  xorc02aa1n02x5               g029(.a(\a[10] ), .b(\b[9] ), .out0(new_n125));
  xnbna2aa1n03x5               g030(.a(new_n125), .b(new_n124), .c(new_n97), .out0(\s[10] ));
  norp02aa1n02x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  160nm_ficinv00aa1n08x5       g032(.clk(new_n127), .clkout(new_n128));
  nanp02aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(new_n124), .b(new_n97), .o1(new_n130));
  oaoi03aa1n02x5               g035(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n131));
  aoi012aa1n02x5               g036(.a(new_n131), .b(new_n130), .c(new_n125), .o1(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n132), .b(new_n128), .c(new_n129), .out0(\s[11] ));
  norb02aa1n02x5               g038(.a(new_n129), .b(new_n127), .out0(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n131), .c(new_n130), .d(new_n125), .o1(new_n135));
  norp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  nona22aa1n02x4               g043(.a(new_n135), .b(new_n138), .c(new_n127), .out0(new_n139));
  aobi12aa1n02x5               g044(.a(new_n138), .b(new_n135), .c(new_n128), .out0(new_n140));
  norb02aa1n02x5               g045(.a(new_n139), .b(new_n140), .out0(\s[12] ));
  nano23aa1n02x4               g046(.a(new_n127), .b(new_n136), .c(new_n137), .d(new_n129), .out0(new_n142));
  and003aa1n02x5               g047(.a(new_n142), .b(new_n125), .c(new_n123), .o(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n122), .c(new_n108), .d(new_n116), .o1(new_n144));
  aoi012aa1n02x5               g049(.a(new_n136), .b(new_n127), .c(new_n137), .o1(new_n145));
  aobi12aa1n02x5               g050(.a(new_n145), .b(new_n142), .c(new_n131), .out0(new_n146));
  xorc02aa1n02x5               g051(.a(\a[13] ), .b(\b[12] ), .out0(new_n147));
  xnbna2aa1n03x5               g052(.a(new_n147), .b(new_n144), .c(new_n146), .out0(\s[13] ));
  orn002aa1n02x5               g053(.a(\a[13] ), .b(\b[12] ), .o(new_n149));
  aob012aa1n02x5               g054(.a(new_n147), .b(new_n144), .c(new_n146), .out0(new_n150));
  xorc02aa1n02x5               g055(.a(\a[14] ), .b(\b[13] ), .out0(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n151), .b(new_n150), .c(new_n149), .out0(\s[14] ));
  nanp02aa1n02x5               g057(.a(new_n151), .b(new_n147), .o1(new_n153));
  oaoi03aa1n02x5               g058(.a(\a[14] ), .b(\b[13] ), .c(new_n149), .o1(new_n154));
  160nm_ficinv00aa1n08x5       g059(.clk(new_n154), .clkout(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n153), .c(new_n144), .d(new_n146), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g062(.a(\b[14] ), .b(\a[15] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  norp02aa1n02x5               g064(.a(\b[15] ), .b(\a[16] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[15] ), .b(\a[16] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  aoi112aa1n02x5               g067(.a(new_n162), .b(new_n158), .c(new_n156), .d(new_n159), .o1(new_n163));
  aoai13aa1n02x5               g068(.a(new_n162), .b(new_n158), .c(new_n156), .d(new_n159), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(\s[16] ));
  nano23aa1n02x4               g070(.a(new_n158), .b(new_n160), .c(new_n161), .d(new_n159), .out0(new_n166));
  nanp03aa1n02x5               g071(.a(new_n166), .b(new_n147), .c(new_n151), .o1(new_n167));
  nano32aa1n02x4               g072(.a(new_n167), .b(new_n142), .c(new_n125), .d(new_n123), .out0(new_n168));
  aoai13aa1n02x5               g073(.a(new_n168), .b(new_n122), .c(new_n108), .d(new_n116), .o1(new_n169));
  160nm_fiao0012aa1n02p5x5     g074(.a(new_n160), .b(new_n158), .c(new_n161), .o(new_n170));
  aoi012aa1n02x5               g075(.a(new_n170), .b(new_n166), .c(new_n154), .o1(new_n171));
  160nm_ficinv00aa1n08x5       g076(.clk(new_n171), .clkout(new_n172));
  oab012aa1n02x4               g077(.a(new_n172), .b(new_n146), .c(new_n167), .out0(new_n173));
  nanp02aa1n02x5               g078(.a(new_n169), .b(new_n173), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g080(.clk(\a[18] ), .clkout(new_n176));
  160nm_ficinv00aa1n08x5       g081(.clk(\a[17] ), .clkout(new_n177));
  160nm_ficinv00aa1n08x5       g082(.clk(\b[16] ), .clkout(new_n178));
  oaoi03aa1n02x5               g083(.a(new_n177), .b(new_n178), .c(new_n174), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[17] ), .c(new_n176), .out0(\s[18] ));
  xroi22aa1d04x5               g085(.a(new_n177), .b(\b[16] ), .c(new_n176), .d(\b[17] ), .out0(new_n181));
  oai022aa1n02x5               g086(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n182));
  oaib12aa1n02x5               g087(.a(new_n182), .b(new_n176), .c(\b[17] ), .out0(new_n183));
  160nm_ficinv00aa1n08x5       g088(.clk(new_n183), .clkout(new_n184));
  norp02aa1n02x5               g089(.a(\b[18] ), .b(\a[19] ), .o1(new_n185));
  nanp02aa1n02x5               g090(.a(\b[18] ), .b(\a[19] ), .o1(new_n186));
  norb02aa1n02x5               g091(.a(new_n186), .b(new_n185), .out0(new_n187));
  aoai13aa1n02x5               g092(.a(new_n187), .b(new_n184), .c(new_n174), .d(new_n181), .o1(new_n188));
  aoi112aa1n02x5               g093(.a(new_n187), .b(new_n184), .c(new_n174), .d(new_n181), .o1(new_n189));
  norb02aa1n02x5               g094(.a(new_n188), .b(new_n189), .out0(\s[19] ));
  xnrc02aa1n02x5               g095(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g096(.a(\b[19] ), .b(\a[20] ), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(\b[19] ), .b(\a[20] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  nona22aa1n02x4               g099(.a(new_n188), .b(new_n194), .c(new_n185), .out0(new_n195));
  160nm_ficinv00aa1n08x5       g100(.clk(new_n194), .clkout(new_n196));
  oaoi13aa1n02x5               g101(.a(new_n196), .b(new_n188), .c(\a[19] ), .d(\b[18] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n195), .b(new_n197), .out0(\s[20] ));
  nano23aa1n02x4               g103(.a(new_n185), .b(new_n192), .c(new_n193), .d(new_n186), .out0(new_n199));
  nanp02aa1n02x5               g104(.a(new_n181), .b(new_n199), .o1(new_n200));
  nona23aa1n02x4               g105(.a(new_n193), .b(new_n186), .c(new_n185), .d(new_n192), .out0(new_n201));
  aoi012aa1n02x5               g106(.a(new_n192), .b(new_n185), .c(new_n193), .o1(new_n202));
  oai012aa1n02x5               g107(.a(new_n202), .b(new_n201), .c(new_n183), .o1(new_n203));
  160nm_ficinv00aa1n08x5       g108(.clk(new_n203), .clkout(new_n204));
  aoai13aa1n02x5               g109(.a(new_n204), .b(new_n200), .c(new_n169), .d(new_n173), .o1(new_n205));
  xorb03aa1n02x5               g110(.a(new_n205), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g111(.a(\b[20] ), .b(\a[21] ), .o1(new_n207));
  xorc02aa1n02x5               g112(.a(\a[21] ), .b(\b[20] ), .out0(new_n208));
  xorc02aa1n02x5               g113(.a(\a[22] ), .b(\b[21] ), .out0(new_n209));
  aoi112aa1n02x5               g114(.a(new_n207), .b(new_n209), .c(new_n205), .d(new_n208), .o1(new_n210));
  aoai13aa1n02x5               g115(.a(new_n209), .b(new_n207), .c(new_n205), .d(new_n208), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n210), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g117(.clk(\a[21] ), .clkout(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(\a[22] ), .clkout(new_n214));
  xroi22aa1d04x5               g119(.a(new_n213), .b(\b[20] ), .c(new_n214), .d(\b[21] ), .out0(new_n215));
  nanp03aa1n02x5               g120(.a(new_n215), .b(new_n181), .c(new_n199), .o1(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(\b[21] ), .clkout(new_n217));
  oaoi03aa1n02x5               g122(.a(new_n214), .b(new_n217), .c(new_n207), .o1(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n218), .clkout(new_n219));
  aoi012aa1n02x5               g124(.a(new_n219), .b(new_n203), .c(new_n215), .o1(new_n220));
  aoai13aa1n02x5               g125(.a(new_n220), .b(new_n216), .c(new_n169), .d(new_n173), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g127(.a(\b[22] ), .b(\a[23] ), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[23] ), .b(\b[22] ), .out0(new_n224));
  xorc02aa1n02x5               g129(.a(\a[24] ), .b(\b[23] ), .out0(new_n225));
  aoi112aa1n02x5               g130(.a(new_n223), .b(new_n225), .c(new_n221), .d(new_n224), .o1(new_n226));
  aoai13aa1n02x5               g131(.a(new_n225), .b(new_n223), .c(new_n221), .d(new_n224), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n227), .b(new_n226), .out0(\s[24] ));
  nano23aa1n02x4               g133(.a(new_n102), .b(new_n104), .c(new_n105), .d(new_n103), .out0(new_n229));
  nanb02aa1n02x5               g134(.a(new_n101), .b(new_n229), .out0(new_n230));
  nano23aa1n02x4               g135(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n231));
  nona22aa1n02x4               g136(.a(new_n231), .b(new_n114), .c(new_n115), .out0(new_n232));
  aobi12aa1n02x5               g137(.a(new_n117), .b(new_n231), .c(new_n121), .out0(new_n233));
  aoai13aa1n02x5               g138(.a(new_n233), .b(new_n232), .c(new_n230), .d(new_n107), .o1(new_n234));
  oai012aa1n02x5               g139(.a(new_n171), .b(new_n146), .c(new_n167), .o1(new_n235));
  and002aa1n02x5               g140(.a(new_n225), .b(new_n224), .o(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n236), .clkout(new_n237));
  nano32aa1n02x4               g142(.a(new_n237), .b(new_n215), .c(new_n181), .d(new_n199), .out0(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n235), .c(new_n234), .d(new_n168), .o1(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n202), .clkout(new_n240));
  aoai13aa1n02x5               g145(.a(new_n215), .b(new_n240), .c(new_n199), .d(new_n184), .o1(new_n241));
  aoi112aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n242));
  oab012aa1n02x4               g147(.a(new_n242), .b(\a[24] ), .c(\b[23] ), .out0(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n237), .c(new_n241), .d(new_n218), .o1(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n244), .clkout(new_n245));
  xnrc02aa1n02x5               g150(.a(\b[24] ), .b(\a[25] ), .out0(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n246), .clkout(new_n247));
  xnbna2aa1n03x5               g152(.a(new_n247), .b(new_n239), .c(new_n245), .out0(\s[25] ));
  norp02aa1n02x5               g153(.a(\b[24] ), .b(\a[25] ), .o1(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n249), .clkout(new_n250));
  aoai13aa1n02x5               g155(.a(new_n247), .b(new_n244), .c(new_n174), .d(new_n238), .o1(new_n251));
  xnrc02aa1n02x5               g156(.a(\b[25] ), .b(\a[26] ), .out0(new_n252));
  nanp03aa1n02x5               g157(.a(new_n251), .b(new_n250), .c(new_n252), .o1(new_n253));
  aoi012aa1n02x5               g158(.a(new_n252), .b(new_n251), .c(new_n250), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n253), .b(new_n254), .out0(\s[26] ));
  norp02aa1n02x5               g160(.a(new_n252), .b(new_n246), .o1(new_n256));
  nano22aa1n02x4               g161(.a(new_n216), .b(new_n236), .c(new_n256), .out0(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n235), .c(new_n234), .d(new_n168), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(new_n244), .b(new_n256), .o1(new_n259));
  oao003aa1n02x5               g164(.a(\a[26] ), .b(\b[25] ), .c(new_n250), .carry(new_n260));
  xorc02aa1n02x5               g165(.a(\a[27] ), .b(\b[26] ), .out0(new_n261));
  160nm_ficinv00aa1n08x5       g166(.clk(new_n261), .clkout(new_n262));
  aoi013aa1n02x4               g167(.a(new_n262), .b(new_n258), .c(new_n259), .d(new_n260), .o1(new_n263));
  aobi12aa1n02x5               g168(.a(new_n257), .b(new_n169), .c(new_n173), .out0(new_n264));
  aoai13aa1n02x5               g169(.a(new_n236), .b(new_n219), .c(new_n203), .d(new_n215), .o1(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(new_n256), .clkout(new_n266));
  aoai13aa1n02x5               g171(.a(new_n260), .b(new_n266), .c(new_n265), .d(new_n243), .o1(new_n267));
  norp03aa1n02x5               g172(.a(new_n267), .b(new_n264), .c(new_n261), .o1(new_n268));
  norp02aa1n02x5               g173(.a(new_n263), .b(new_n268), .o1(\s[27] ));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n270), .clkout(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[27] ), .b(\a[28] ), .out0(new_n272));
  nano22aa1n02x4               g177(.a(new_n263), .b(new_n271), .c(new_n272), .out0(new_n273));
  oai012aa1n02x5               g178(.a(new_n261), .b(new_n267), .c(new_n264), .o1(new_n274));
  aoi012aa1n02x5               g179(.a(new_n272), .b(new_n274), .c(new_n271), .o1(new_n275));
  norp02aa1n02x5               g180(.a(new_n275), .b(new_n273), .o1(\s[28] ));
  norb02aa1n02x5               g181(.a(new_n261), .b(new_n272), .out0(new_n277));
  oai012aa1n02x5               g182(.a(new_n277), .b(new_n267), .c(new_n264), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[28] ), .b(\b[27] ), .c(new_n271), .carry(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[28] ), .b(\a[29] ), .out0(new_n280));
  aoi012aa1n02x5               g185(.a(new_n280), .b(new_n278), .c(new_n279), .o1(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n277), .clkout(new_n282));
  aoi013aa1n02x4               g187(.a(new_n282), .b(new_n258), .c(new_n259), .d(new_n260), .o1(new_n283));
  nano22aa1n02x4               g188(.a(new_n283), .b(new_n279), .c(new_n280), .out0(new_n284));
  norp02aa1n02x5               g189(.a(new_n281), .b(new_n284), .o1(\s[29] ));
  xorb03aa1n02x5               g190(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g191(.a(new_n261), .b(new_n280), .c(new_n272), .out0(new_n287));
  oai012aa1n02x5               g192(.a(new_n287), .b(new_n267), .c(new_n264), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[29] ), .b(\b[28] ), .c(new_n279), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[29] ), .b(\a[30] ), .out0(new_n290));
  aoi012aa1n02x5               g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  160nm_ficinv00aa1n08x5       g196(.clk(new_n287), .clkout(new_n292));
  aoi013aa1n02x4               g197(.a(new_n292), .b(new_n258), .c(new_n259), .d(new_n260), .o1(new_n293));
  nano22aa1n02x4               g198(.a(new_n293), .b(new_n289), .c(new_n290), .out0(new_n294));
  norp02aa1n02x5               g199(.a(new_n291), .b(new_n294), .o1(\s[30] ));
  xnrc02aa1n02x5               g200(.a(\b[30] ), .b(\a[31] ), .out0(new_n296));
  norb02aa1n02x5               g201(.a(new_n287), .b(new_n290), .out0(new_n297));
  160nm_ficinv00aa1n08x5       g202(.clk(new_n297), .clkout(new_n298));
  aoi013aa1n02x4               g203(.a(new_n298), .b(new_n258), .c(new_n259), .d(new_n260), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[30] ), .b(\b[29] ), .c(new_n289), .carry(new_n300));
  nano22aa1n02x4               g205(.a(new_n299), .b(new_n296), .c(new_n300), .out0(new_n301));
  oai012aa1n02x5               g206(.a(new_n297), .b(new_n267), .c(new_n264), .o1(new_n302));
  aoi012aa1n02x5               g207(.a(new_n296), .b(new_n302), .c(new_n300), .o1(new_n303));
  norp02aa1n02x5               g208(.a(new_n303), .b(new_n301), .o1(\s[31] ));
  xnrb03aa1n02x5               g209(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g210(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n306));
  xorb03aa1n02x5               g211(.a(new_n306), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g212(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g213(.a(new_n118), .b(new_n119), .c(new_n108), .o1(new_n309));
  xnrb03aa1n02x5               g214(.a(new_n309), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g215(.a(\a[6] ), .b(\b[5] ), .c(new_n309), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g217(.a(new_n111), .b(new_n311), .c(new_n112), .o1(new_n313));
  xnrb03aa1n02x5               g218(.a(new_n313), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g219(.a(new_n234), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


