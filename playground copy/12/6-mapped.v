// Benchmark "adder" written by ABC on Thu Jul 11 11:44:45 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n315, new_n318, new_n320,
    new_n322;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\a[9] ), .clkout(new_n98));
  nanb02aa1n02x5               g003(.a(\b[8] ), .b(new_n98), .out0(new_n99));
  norp02aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nona23aa1n02x4               g008(.a(new_n103), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n104));
  norp02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  nona23aa1n02x4               g013(.a(new_n108), .b(new_n106), .c(new_n105), .d(new_n107), .out0(new_n109));
  norp02aa1n02x5               g014(.a(new_n109), .b(new_n104), .o1(new_n110));
  and002aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o(new_n111));
  160nm_ficinv00aa1n08x5       g016(.clk(\a[3] ), .clkout(new_n112));
  160nm_ficinv00aa1n08x5       g017(.clk(\b[2] ), .clkout(new_n113));
  nanp02aa1n02x5               g018(.a(new_n113), .b(new_n112), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(new_n114), .b(new_n115), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[0] ), .b(\a[1] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  aoi012aa1n02x5               g024(.a(new_n117), .b(new_n118), .c(new_n119), .o1(new_n120));
  oa0022aa1n02x5               g025(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n121));
  oaoi13aa1n02x5               g026(.a(new_n111), .b(new_n121), .c(new_n120), .d(new_n116), .o1(new_n122));
  oai012aa1n02x5               g027(.a(new_n106), .b(new_n107), .c(new_n105), .o1(new_n123));
  aoi012aa1n02x5               g028(.a(new_n100), .b(new_n102), .c(new_n101), .o1(new_n124));
  oai012aa1n02x5               g029(.a(new_n124), .b(new_n104), .c(new_n123), .o1(new_n125));
  xorc02aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n125), .c(new_n122), .d(new_n110), .o1(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n97), .b(new_n127), .c(new_n99), .out0(\s[10] ));
  aoi012aa1n02x5               g033(.a(new_n125), .b(new_n122), .c(new_n110), .o1(new_n129));
  norp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  160nm_ficinv00aa1n08x5       g037(.clk(new_n132), .clkout(new_n133));
  oaoi03aa1n02x5               g038(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n134));
  160nm_ficinv00aa1n08x5       g039(.clk(new_n134), .clkout(new_n135));
  nanp02aa1n02x5               g040(.a(new_n126), .b(new_n97), .o1(new_n136));
  oaoi13aa1n02x5               g041(.a(new_n133), .b(new_n135), .c(new_n129), .d(new_n136), .o1(new_n137));
  oai112aa1n02x5               g042(.a(new_n135), .b(new_n133), .c(new_n129), .d(new_n136), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(\s[11] ));
  norp02aa1n02x5               g044(.a(new_n137), .b(new_n130), .o1(new_n140));
  xnrb03aa1n02x5               g045(.a(new_n140), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g046(.a(\b[12] ), .b(\a[13] ), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(\b[12] ), .b(\a[13] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  norp02aa1n02x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nano23aa1n02x4               g051(.a(new_n130), .b(new_n145), .c(new_n146), .d(new_n131), .out0(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n136), .out0(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n125), .c(new_n122), .d(new_n110), .o1(new_n149));
  aoi012aa1n02x5               g054(.a(new_n145), .b(new_n130), .c(new_n146), .o1(new_n150));
  aobi12aa1n02x5               g055(.a(new_n150), .b(new_n147), .c(new_n134), .out0(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n144), .b(new_n149), .c(new_n151), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g057(.clk(new_n142), .clkout(new_n153));
  aob012aa1n02x5               g058(.a(new_n144), .b(new_n149), .c(new_n151), .out0(new_n154));
  norp02aa1n02x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nanb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(new_n157));
  xobna2aa1n03x5               g062(.a(new_n157), .b(new_n154), .c(new_n153), .out0(\s[14] ));
  nano23aa1n02x4               g063(.a(new_n142), .b(new_n155), .c(new_n156), .d(new_n143), .out0(new_n159));
  160nm_ficinv00aa1n08x5       g064(.clk(new_n159), .clkout(new_n160));
  oaoi03aa1n02x5               g065(.a(\a[14] ), .b(\b[13] ), .c(new_n153), .o1(new_n161));
  160nm_ficinv00aa1n08x5       g066(.clk(new_n161), .clkout(new_n162));
  aoai13aa1n02x5               g067(.a(new_n162), .b(new_n160), .c(new_n149), .d(new_n151), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  norp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  160nm_ficinv00aa1n08x5       g072(.clk(new_n167), .clkout(new_n168));
  nanp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  aoi122aa1n02x5               g074(.a(new_n165), .b(new_n169), .c(new_n168), .d(new_n163), .e(new_n166), .o1(new_n170));
  aoi012aa1n02x5               g075(.a(new_n165), .b(new_n163), .c(new_n166), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n167), .b(new_n169), .out0(new_n172));
  norp02aa1n02x5               g077(.a(new_n171), .b(new_n172), .o1(new_n173));
  norp02aa1n02x5               g078(.a(new_n173), .b(new_n170), .o1(\s[16] ));
  nano23aa1n02x4               g079(.a(new_n165), .b(new_n167), .c(new_n169), .d(new_n166), .out0(new_n175));
  nanp02aa1n02x5               g080(.a(new_n175), .b(new_n159), .o1(new_n176));
  nano32aa1n02x4               g081(.a(new_n176), .b(new_n147), .c(new_n126), .d(new_n97), .out0(new_n177));
  160nm_ficinv00aa1n08x5       g082(.clk(new_n177), .clkout(new_n178));
  nanp02aa1n02x5               g083(.a(new_n175), .b(new_n161), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(new_n165), .b(new_n169), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(new_n147), .b(new_n134), .o1(new_n181));
  aoi012aa1n02x5               g086(.a(new_n176), .b(new_n181), .c(new_n150), .o1(new_n182));
  nano32aa1n02x4               g087(.a(new_n182), .b(new_n180), .c(new_n179), .d(new_n168), .out0(new_n183));
  oai012aa1n02x5               g088(.a(new_n183), .b(new_n129), .c(new_n178), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[18] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[17] ), .clkout(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(\b[16] ), .clkout(new_n188));
  oaoi03aa1n02x5               g093(.a(new_n187), .b(new_n188), .c(new_n184), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[17] ), .c(new_n186), .out0(\s[18] ));
  aoai13aa1n02x5               g095(.a(new_n177), .b(new_n125), .c(new_n122), .d(new_n110), .o1(new_n191));
  xroi22aa1d04x5               g096(.a(new_n187), .b(\b[16] ), .c(new_n186), .d(\b[17] ), .out0(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(new_n192), .clkout(new_n193));
  oai022aa1n02x5               g098(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n194));
  oaib12aa1n02x5               g099(.a(new_n194), .b(new_n186), .c(\b[17] ), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n193), .c(new_n183), .d(new_n191), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  xorc02aa1n02x5               g104(.a(\a[19] ), .b(\b[18] ), .out0(new_n200));
  xnrc02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .out0(new_n201));
  160nm_ficinv00aa1n08x5       g106(.clk(new_n201), .clkout(new_n202));
  aoi112aa1n02x5               g107(.a(new_n199), .b(new_n202), .c(new_n196), .d(new_n200), .o1(new_n203));
  160nm_ficinv00aa1n08x5       g108(.clk(new_n199), .clkout(new_n204));
  nanp02aa1n02x5               g109(.a(new_n196), .b(new_n200), .o1(new_n205));
  aoi012aa1n02x5               g110(.a(new_n201), .b(new_n205), .c(new_n204), .o1(new_n206));
  norp02aa1n02x5               g111(.a(new_n206), .b(new_n203), .o1(\s[20] ));
  xnrc02aa1n02x5               g112(.a(\b[18] ), .b(\a[19] ), .out0(new_n208));
  nona22aa1n02x4               g113(.a(new_n192), .b(new_n208), .c(new_n201), .out0(new_n209));
  oao003aa1n02x5               g114(.a(\a[20] ), .b(\b[19] ), .c(new_n204), .carry(new_n210));
  oai013aa1n02x4               g115(.a(new_n210), .b(new_n208), .c(new_n201), .d(new_n195), .o1(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(new_n211), .clkout(new_n212));
  aoai13aa1n02x5               g117(.a(new_n212), .b(new_n209), .c(new_n183), .d(new_n191), .o1(new_n213));
  xorb03aa1n02x5               g118(.a(new_n213), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  nanp02aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  norp02aa1n02x5               g122(.a(\b[21] ), .b(\a[22] ), .o1(new_n218));
  nanp02aa1n02x5               g123(.a(\b[21] ), .b(\a[22] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  aoi112aa1n02x5               g125(.a(new_n215), .b(new_n220), .c(new_n213), .d(new_n217), .o1(new_n221));
  160nm_ficinv00aa1n08x5       g126(.clk(new_n215), .clkout(new_n222));
  nanp02aa1n02x5               g127(.a(new_n213), .b(new_n217), .o1(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(new_n220), .clkout(new_n224));
  aoi012aa1n02x5               g129(.a(new_n224), .b(new_n223), .c(new_n222), .o1(new_n225));
  norp02aa1n02x5               g130(.a(new_n225), .b(new_n221), .o1(\s[22] ));
  nano23aa1n02x4               g131(.a(new_n215), .b(new_n218), .c(new_n219), .d(new_n216), .out0(new_n227));
  oaoi03aa1n02x5               g132(.a(\a[22] ), .b(\b[21] ), .c(new_n222), .o1(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n211), .c(new_n227), .o1(new_n229));
  nona23aa1n02x4               g134(.a(new_n219), .b(new_n216), .c(new_n215), .d(new_n218), .out0(new_n230));
  nona32aa1n02x4               g135(.a(new_n192), .b(new_n230), .c(new_n201), .d(new_n208), .out0(new_n231));
  aoai13aa1n02x5               g136(.a(new_n229), .b(new_n231), .c(new_n183), .d(new_n191), .o1(new_n232));
  xorb03aa1n02x5               g137(.a(new_n232), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  nanp02aa1n02x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n235), .b(new_n234), .out0(new_n236));
  norp02aa1n02x5               g141(.a(\b[23] ), .b(\a[24] ), .o1(new_n237));
  nanp02aa1n02x5               g142(.a(\b[23] ), .b(\a[24] ), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n238), .b(new_n237), .out0(new_n239));
  aoi112aa1n02x5               g144(.a(new_n234), .b(new_n239), .c(new_n232), .d(new_n236), .o1(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n234), .clkout(new_n241));
  nanp02aa1n02x5               g146(.a(new_n232), .b(new_n236), .o1(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n239), .clkout(new_n243));
  aoi012aa1n02x5               g148(.a(new_n243), .b(new_n242), .c(new_n241), .o1(new_n244));
  norp02aa1n02x5               g149(.a(new_n244), .b(new_n240), .o1(\s[24] ));
  nano23aa1n02x4               g150(.a(new_n234), .b(new_n237), .c(new_n238), .d(new_n235), .out0(new_n246));
  nanp02aa1n02x5               g151(.a(new_n246), .b(new_n227), .o1(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n247), .clkout(new_n248));
  nona23aa1n02x4               g153(.a(new_n248), .b(new_n192), .c(new_n208), .d(new_n201), .out0(new_n249));
  oaoi03aa1n02x5               g154(.a(\a[24] ), .b(\b[23] ), .c(new_n241), .o1(new_n250));
  aoi012aa1n02x5               g155(.a(new_n250), .b(new_n246), .c(new_n228), .o1(new_n251));
  aobi12aa1n02x5               g156(.a(new_n251), .b(new_n248), .c(new_n211), .out0(new_n252));
  aoai13aa1n02x5               g157(.a(new_n252), .b(new_n249), .c(new_n183), .d(new_n191), .o1(new_n253));
  xorb03aa1n02x5               g158(.a(new_n253), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g159(.a(\b[24] ), .b(\a[25] ), .o1(new_n255));
  xorc02aa1n02x5               g160(.a(\a[25] ), .b(\b[24] ), .out0(new_n256));
  xorc02aa1n02x5               g161(.a(\a[26] ), .b(\b[25] ), .out0(new_n257));
  aoi112aa1n02x5               g162(.a(new_n255), .b(new_n257), .c(new_n253), .d(new_n256), .o1(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n255), .clkout(new_n259));
  nanp02aa1n02x5               g164(.a(new_n253), .b(new_n256), .o1(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n257), .clkout(new_n261));
  aoi012aa1n02x5               g166(.a(new_n261), .b(new_n260), .c(new_n259), .o1(new_n262));
  norp02aa1n02x5               g167(.a(new_n262), .b(new_n258), .o1(\s[26] ));
  and002aa1n02x5               g168(.a(new_n257), .b(new_n256), .o(new_n264));
  nano32aa1n02x4               g169(.a(new_n209), .b(new_n264), .c(new_n227), .d(new_n246), .out0(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(new_n265), .clkout(new_n266));
  nanp02aa1n02x5               g171(.a(new_n188), .b(new_n187), .o1(new_n267));
  oaoi03aa1n02x5               g172(.a(\a[18] ), .b(\b[17] ), .c(new_n267), .o1(new_n268));
  nanb03aa1n02x5               g173(.a(new_n201), .b(new_n268), .c(new_n200), .out0(new_n269));
  aoai13aa1n02x5               g174(.a(new_n251), .b(new_n247), .c(new_n269), .d(new_n210), .o1(new_n270));
  oao003aa1n02x5               g175(.a(\a[26] ), .b(\b[25] ), .c(new_n259), .carry(new_n271));
  aobi12aa1n02x5               g176(.a(new_n271), .b(new_n270), .c(new_n264), .out0(new_n272));
  aoai13aa1n02x5               g177(.a(new_n272), .b(new_n266), .c(new_n191), .d(new_n183), .o1(new_n273));
  xorb03aa1n02x5               g178(.a(new_n273), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n275), .clkout(new_n276));
  xorc02aa1n02x5               g181(.a(\a[28] ), .b(\b[27] ), .out0(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n277), .clkout(new_n278));
  160nm_ficinv00aa1n08x5       g183(.clk(new_n264), .clkout(new_n279));
  oai012aa1n02x5               g184(.a(new_n271), .b(new_n252), .c(new_n279), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  aoai13aa1n02x5               g186(.a(new_n281), .b(new_n280), .c(new_n184), .d(new_n265), .o1(new_n282));
  aoi012aa1n02x5               g187(.a(new_n278), .b(new_n282), .c(new_n276), .o1(new_n283));
  aoi112aa1n02x5               g188(.a(new_n275), .b(new_n277), .c(new_n273), .d(new_n281), .o1(new_n284));
  norp02aa1n02x5               g189(.a(new_n283), .b(new_n284), .o1(\s[28] ));
  nano22aa1n02x4               g190(.a(new_n278), .b(new_n276), .c(new_n281), .out0(new_n286));
  aoai13aa1n02x5               g191(.a(new_n286), .b(new_n280), .c(new_n184), .d(new_n265), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[28] ), .b(\b[27] ), .c(new_n276), .carry(new_n288));
  xorc02aa1n02x5               g193(.a(\a[29] ), .b(\b[28] ), .out0(new_n289));
  160nm_ficinv00aa1n08x5       g194(.clk(new_n289), .clkout(new_n290));
  aoi012aa1n02x5               g195(.a(new_n290), .b(new_n287), .c(new_n288), .o1(new_n291));
  160nm_ficinv00aa1n08x5       g196(.clk(new_n288), .clkout(new_n292));
  aoi112aa1n02x5               g197(.a(new_n289), .b(new_n292), .c(new_n273), .d(new_n286), .o1(new_n293));
  norp02aa1n02x5               g198(.a(new_n291), .b(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n118), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano32aa1n02x4               g200(.a(new_n290), .b(new_n277), .c(new_n281), .d(new_n276), .out0(new_n296));
  aoai13aa1n02x5               g201(.a(new_n296), .b(new_n280), .c(new_n184), .d(new_n265), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n288), .carry(new_n298));
  xorc02aa1n02x5               g203(.a(\a[30] ), .b(\b[29] ), .out0(new_n299));
  160nm_ficinv00aa1n08x5       g204(.clk(new_n299), .clkout(new_n300));
  aoi012aa1n02x5               g205(.a(new_n300), .b(new_n297), .c(new_n298), .o1(new_n301));
  160nm_ficinv00aa1n08x5       g206(.clk(new_n298), .clkout(new_n302));
  aoi112aa1n02x5               g207(.a(new_n299), .b(new_n302), .c(new_n273), .d(new_n296), .o1(new_n303));
  norp02aa1n02x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  160nm_ficinv00aa1n08x5       g210(.clk(new_n305), .clkout(new_n306));
  and003aa1n02x5               g211(.a(new_n286), .b(new_n299), .c(new_n289), .o(new_n307));
  oao003aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .c(new_n298), .carry(new_n308));
  160nm_ficinv00aa1n08x5       g213(.clk(new_n308), .clkout(new_n309));
  aoi112aa1n02x5               g214(.a(new_n306), .b(new_n309), .c(new_n273), .d(new_n307), .o1(new_n310));
  aoai13aa1n02x5               g215(.a(new_n307), .b(new_n280), .c(new_n184), .d(new_n265), .o1(new_n311));
  aoi012aa1n02x5               g216(.a(new_n305), .b(new_n311), .c(new_n308), .o1(new_n312));
  norp02aa1n02x5               g217(.a(new_n312), .b(new_n310), .o1(\s[31] ));
  xnbna2aa1n03x5               g218(.a(new_n120), .b(new_n114), .c(new_n115), .out0(\s[3] ));
  oaoi03aa1n02x5               g219(.a(\a[3] ), .b(\b[2] ), .c(new_n120), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g221(.a(new_n122), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g222(.a(new_n108), .b(new_n122), .c(new_n107), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g224(.a(new_n123), .b(new_n109), .c(new_n122), .out0(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g226(.a(new_n102), .b(new_n320), .c(new_n103), .o1(new_n322));
  xnrb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g228(.a(new_n129), .b(\b[8] ), .c(new_n98), .out0(\s[9] ));
endmodule


