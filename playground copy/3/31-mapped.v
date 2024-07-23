// Benchmark "adder" written by ABC on Wed Jul 10 16:55:05 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n314, new_n317, new_n319, new_n321;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  and002aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o(new_n98));
  and002aa1n02x5               g003(.a(\b[3] ), .b(\a[4] ), .o(new_n99));
  xnrc02aa1n02x5               g004(.a(\b[2] ), .b(\a[3] ), .out0(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  oai012aa1n02x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  oa0022aa1n02x5               g009(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n105));
  oaoi13aa1n02x5               g010(.a(new_n99), .b(new_n105), .c(new_n100), .d(new_n104), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[7] ), .b(\a[8] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nona23aa1n02x4               g015(.a(new_n110), .b(new_n108), .c(new_n107), .d(new_n109), .out0(new_n111));
  xnrc02aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .out0(new_n112));
  xorc02aa1n02x5               g017(.a(\a[5] ), .b(\b[4] ), .out0(new_n113));
  norb03aa1n02x5               g018(.a(new_n113), .b(new_n111), .c(new_n112), .out0(new_n114));
  aoi112aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n115));
  nano23aa1n02x4               g020(.a(new_n107), .b(new_n109), .c(new_n110), .d(new_n108), .out0(new_n116));
  and002aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .o(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\a[5] ), .clkout(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\a[6] ), .clkout(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\b[4] ), .clkout(new_n120));
  aboi22aa1n03x5               g025(.a(\b[5] ), .b(new_n119), .c(new_n118), .d(new_n120), .out0(new_n121));
  nona22aa1n02x4               g026(.a(new_n116), .b(new_n117), .c(new_n121), .out0(new_n122));
  nona22aa1n02x4               g027(.a(new_n122), .b(new_n115), .c(new_n107), .out0(new_n123));
  norp02aa1n02x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  norp02aa1n02x5               g029(.a(new_n98), .b(new_n124), .o1(new_n125));
  160nm_ficinv00aa1n08x5       g030(.clk(new_n125), .clkout(new_n126));
  aoi112aa1n02x5               g031(.a(new_n123), .b(new_n126), .c(new_n106), .d(new_n114), .o1(new_n127));
  oai012aa1n02x5               g032(.a(new_n97), .b(new_n127), .c(new_n98), .o1(new_n128));
  orn003aa1n02x5               g033(.a(new_n127), .b(new_n97), .c(new_n98), .o(new_n129));
  nanp02aa1n02x5               g034(.a(new_n129), .b(new_n128), .o1(\s[10] ));
  nanp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xobna2aa1n03x5               g039(.a(new_n134), .b(new_n128), .c(new_n131), .out0(\s[11] ));
  xorc02aa1n02x5               g040(.a(\a[12] ), .b(\b[11] ), .out0(new_n136));
  aoi113aa1n02x5               g041(.a(new_n136), .b(new_n132), .c(new_n128), .d(new_n134), .e(new_n131), .o1(new_n137));
  nanp03aa1n02x5               g042(.a(new_n128), .b(new_n131), .c(new_n134), .o1(new_n138));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n136), .clkout(new_n139));
  oaoi13aa1n02x5               g044(.a(new_n139), .b(new_n138), .c(\a[11] ), .d(\b[10] ), .o1(new_n140));
  norp02aa1n02x5               g045(.a(new_n140), .b(new_n137), .o1(\s[12] ));
  xnrc02aa1n02x5               g046(.a(\b[12] ), .b(\a[13] ), .out0(new_n142));
  160nm_ficinv00aa1n08x5       g047(.clk(new_n142), .clkout(new_n143));
  160nm_ficinv00aa1n08x5       g048(.clk(\a[10] ), .clkout(new_n144));
  160nm_ficinv00aa1n08x5       g049(.clk(\b[9] ), .clkout(new_n145));
  aoi112aa1n02x5               g050(.a(new_n98), .b(new_n124), .c(new_n144), .d(new_n145), .o1(new_n146));
  nano22aa1n02x4               g051(.a(new_n132), .b(new_n131), .c(new_n133), .out0(new_n147));
  nanp03aa1n02x5               g052(.a(new_n146), .b(new_n147), .c(new_n136), .o1(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(new_n148), .clkout(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n123), .c(new_n106), .d(new_n114), .o1(new_n150));
  norp02aa1n02x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  aoi112aa1n02x5               g056(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n152));
  oai022aa1n02x5               g057(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n153));
  aoi113aa1n02x5               g058(.a(new_n152), .b(new_n151), .c(new_n147), .d(new_n136), .e(new_n153), .o1(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n143), .b(new_n150), .c(new_n154), .out0(\s[13] ));
  orn002aa1n02x5               g060(.a(\a[13] ), .b(\b[12] ), .o(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n142), .c(new_n150), .d(new_n154), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .out0(new_n159));
  norp02aa1n02x5               g064(.a(new_n159), .b(new_n142), .o1(new_n160));
  160nm_ficinv00aa1n08x5       g065(.clk(new_n160), .clkout(new_n161));
  oao003aa1n02x5               g066(.a(\a[14] ), .b(\b[13] ), .c(new_n156), .carry(new_n162));
  aoai13aa1n02x5               g067(.a(new_n162), .b(new_n161), .c(new_n150), .d(new_n154), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  norp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  aoi112aa1n02x5               g074(.a(new_n169), .b(new_n165), .c(new_n163), .d(new_n166), .o1(new_n170));
  aoai13aa1n02x5               g075(.a(new_n169), .b(new_n165), .c(new_n163), .d(new_n166), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(\s[16] ));
  nano23aa1n02x4               g077(.a(new_n165), .b(new_n167), .c(new_n168), .d(new_n166), .out0(new_n173));
  nona22aa1n02x4               g078(.a(new_n173), .b(new_n159), .c(new_n142), .out0(new_n174));
  norp02aa1n02x5               g079(.a(new_n174), .b(new_n148), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n123), .c(new_n106), .d(new_n114), .o1(new_n176));
  nanp03aa1n02x5               g081(.a(new_n147), .b(new_n136), .c(new_n153), .o1(new_n177));
  nona22aa1n02x4               g082(.a(new_n177), .b(new_n152), .c(new_n151), .out0(new_n178));
  nona23aa1n02x4               g083(.a(new_n168), .b(new_n166), .c(new_n165), .d(new_n167), .out0(new_n179));
  norp03aa1n02x5               g084(.a(new_n179), .b(new_n159), .c(new_n142), .o1(new_n180));
  oai012aa1n02x5               g085(.a(new_n168), .b(new_n167), .c(new_n165), .o1(new_n181));
  oai012aa1n02x5               g086(.a(new_n181), .b(new_n162), .c(new_n179), .o1(new_n182));
  aoi012aa1n02x5               g087(.a(new_n182), .b(new_n178), .c(new_n180), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(new_n176), .b(new_n183), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[18] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[17] ), .clkout(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(\b[16] ), .clkout(new_n188));
  oaoi03aa1n02x5               g093(.a(new_n187), .b(new_n188), .c(new_n184), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[17] ), .c(new_n186), .out0(\s[18] ));
  xroi22aa1d04x5               g095(.a(new_n187), .b(\b[16] ), .c(new_n186), .d(\b[17] ), .out0(new_n191));
  oai022aa1n02x5               g096(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n192));
  oaib12aa1n02x5               g097(.a(new_n192), .b(new_n186), .c(\b[17] ), .out0(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(new_n193), .clkout(new_n194));
  norp02aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  aoai13aa1n02x5               g102(.a(new_n197), .b(new_n194), .c(new_n184), .d(new_n191), .o1(new_n198));
  aoi112aa1n02x5               g103(.a(new_n197), .b(new_n194), .c(new_n184), .d(new_n191), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  nona22aa1n02x4               g109(.a(new_n198), .b(new_n204), .c(new_n195), .out0(new_n205));
  orn002aa1n02x5               g110(.a(\a[19] ), .b(\b[18] ), .o(new_n206));
  aobi12aa1n02x5               g111(.a(new_n204), .b(new_n198), .c(new_n206), .out0(new_n207));
  norb02aa1n02x5               g112(.a(new_n205), .b(new_n207), .out0(\s[20] ));
  nano23aa1n02x4               g113(.a(new_n195), .b(new_n202), .c(new_n203), .d(new_n196), .out0(new_n209));
  nanp02aa1n02x5               g114(.a(new_n191), .b(new_n209), .o1(new_n210));
  nona23aa1n02x4               g115(.a(new_n203), .b(new_n196), .c(new_n195), .d(new_n202), .out0(new_n211));
  oaoi03aa1n02x5               g116(.a(\a[20] ), .b(\b[19] ), .c(new_n206), .o1(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  oai012aa1n02x5               g118(.a(new_n213), .b(new_n211), .c(new_n193), .o1(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n214), .clkout(new_n215));
  aoai13aa1n02x5               g120(.a(new_n215), .b(new_n210), .c(new_n176), .d(new_n183), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  xorc02aa1n02x5               g123(.a(\a[21] ), .b(\b[20] ), .out0(new_n219));
  xorc02aa1n02x5               g124(.a(\a[22] ), .b(\b[21] ), .out0(new_n220));
  aoi112aa1n02x5               g125(.a(new_n218), .b(new_n220), .c(new_n216), .d(new_n219), .o1(new_n221));
  aoai13aa1n02x5               g126(.a(new_n220), .b(new_n218), .c(new_n216), .d(new_n219), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g128(.clk(\a[21] ), .clkout(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(\a[22] ), .clkout(new_n225));
  xroi22aa1d04x5               g130(.a(new_n224), .b(\b[20] ), .c(new_n225), .d(\b[21] ), .out0(new_n226));
  nanp03aa1n02x5               g131(.a(new_n226), .b(new_n191), .c(new_n209), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(\b[21] ), .clkout(new_n228));
  oao003aa1n02x5               g133(.a(new_n225), .b(new_n228), .c(new_n218), .carry(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n214), .c(new_n226), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n227), .c(new_n176), .d(new_n183), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  xorc02aa1n02x5               g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  aoi112aa1n02x5               g140(.a(new_n233), .b(new_n235), .c(new_n231), .d(new_n234), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(\s[24] ));
  and002aa1n02x5               g143(.a(new_n235), .b(new_n234), .o(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n239), .clkout(new_n240));
  nano32aa1n02x4               g145(.a(new_n240), .b(new_n226), .c(new_n191), .d(new_n209), .out0(new_n241));
  aoai13aa1n02x5               g146(.a(new_n226), .b(new_n212), .c(new_n209), .d(new_n194), .o1(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n229), .clkout(new_n243));
  oai022aa1n02x5               g148(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n244));
  aob012aa1n02x5               g149(.a(new_n244), .b(\b[23] ), .c(\a[24] ), .out0(new_n245));
  aoai13aa1n02x5               g150(.a(new_n245), .b(new_n240), .c(new_n242), .d(new_n243), .o1(new_n246));
  xorc02aa1n02x5               g151(.a(\a[25] ), .b(\b[24] ), .out0(new_n247));
  aoai13aa1n02x5               g152(.a(new_n247), .b(new_n246), .c(new_n184), .d(new_n241), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(new_n247), .b(new_n246), .c(new_n184), .d(new_n241), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n248), .b(new_n249), .out0(\s[25] ));
  norp02aa1n02x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  xorc02aa1n02x5               g156(.a(\a[26] ), .b(\b[25] ), .out0(new_n252));
  nona22aa1n02x4               g157(.a(new_n248), .b(new_n252), .c(new_n251), .out0(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n251), .clkout(new_n254));
  aobi12aa1n02x5               g159(.a(new_n252), .b(new_n248), .c(new_n254), .out0(new_n255));
  norb02aa1n02x5               g160(.a(new_n253), .b(new_n255), .out0(\s[26] ));
  oai012aa1n02x5               g161(.a(new_n105), .b(new_n100), .c(new_n104), .o1(new_n257));
  nanb02aa1n02x5               g162(.a(new_n99), .b(new_n257), .out0(new_n258));
  nanb03aa1n02x5               g163(.a(new_n112), .b(new_n116), .c(new_n113), .out0(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(new_n117), .clkout(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n121), .clkout(new_n261));
  aoi113aa1n02x5               g166(.a(new_n107), .b(new_n115), .c(new_n116), .d(new_n260), .e(new_n261), .o1(new_n262));
  oai012aa1n02x5               g167(.a(new_n262), .b(new_n258), .c(new_n259), .o1(new_n263));
  oabi12aa1n02x5               g168(.a(new_n182), .b(new_n154), .c(new_n174), .out0(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(\a[25] ), .clkout(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(\a[26] ), .clkout(new_n266));
  xroi22aa1d04x5               g171(.a(new_n265), .b(\b[24] ), .c(new_n266), .d(\b[25] ), .out0(new_n267));
  nano22aa1n02x4               g172(.a(new_n227), .b(new_n239), .c(new_n267), .out0(new_n268));
  aoai13aa1n02x5               g173(.a(new_n268), .b(new_n264), .c(new_n263), .d(new_n175), .o1(new_n269));
  oao003aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .c(new_n254), .carry(new_n270));
  aobi12aa1n02x5               g175(.a(new_n270), .b(new_n246), .c(new_n267), .out0(new_n271));
  norp02aa1n02x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  nanp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  norb02aa1n02x5               g178(.a(new_n273), .b(new_n272), .out0(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n274), .b(new_n271), .c(new_n269), .out0(\s[27] ));
  xorc02aa1n02x5               g180(.a(\a[28] ), .b(\b[27] ), .out0(new_n276));
  aoai13aa1n02x5               g181(.a(new_n239), .b(new_n229), .c(new_n214), .d(new_n226), .o1(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n267), .clkout(new_n278));
  aoai13aa1n02x5               g183(.a(new_n270), .b(new_n278), .c(new_n277), .d(new_n245), .o1(new_n279));
  aoi112aa1n02x5               g184(.a(new_n279), .b(new_n272), .c(new_n184), .d(new_n268), .o1(new_n280));
  nano22aa1n02x4               g185(.a(new_n280), .b(new_n273), .c(new_n276), .out0(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n272), .clkout(new_n282));
  nanp03aa1n02x5               g187(.a(new_n271), .b(new_n269), .c(new_n282), .o1(new_n283));
  aoi012aa1n02x5               g188(.a(new_n276), .b(new_n283), .c(new_n273), .o1(new_n284));
  norp02aa1n02x5               g189(.a(new_n284), .b(new_n281), .o1(\s[28] ));
  160nm_ficinv00aa1n08x5       g190(.clk(new_n268), .clkout(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n176), .c(new_n183), .o1(new_n287));
  and002aa1n02x5               g192(.a(new_n276), .b(new_n274), .o(new_n288));
  oai012aa1n02x5               g193(.a(new_n288), .b(new_n279), .c(new_n287), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[28] ), .b(\a[29] ), .out0(new_n291));
  aoi012aa1n02x5               g196(.a(new_n291), .b(new_n289), .c(new_n290), .o1(new_n292));
  aobi12aa1n02x5               g197(.a(new_n288), .b(new_n271), .c(new_n269), .out0(new_n293));
  nano22aa1n02x4               g198(.a(new_n293), .b(new_n290), .c(new_n291), .out0(new_n294));
  norp02aa1n02x5               g199(.a(new_n292), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g201(.a(new_n291), .b(new_n276), .c(new_n274), .out0(new_n297));
  oai012aa1n02x5               g202(.a(new_n297), .b(new_n279), .c(new_n287), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  aoi012aa1n02x5               g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  aobi12aa1n02x5               g206(.a(new_n297), .b(new_n271), .c(new_n269), .out0(new_n302));
  nano22aa1n02x4               g207(.a(new_n302), .b(new_n299), .c(new_n300), .out0(new_n303));
  norp02aa1n02x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  nano23aa1n02x4               g209(.a(new_n300), .b(new_n291), .c(new_n276), .d(new_n274), .out0(new_n305));
  aobi12aa1n02x5               g210(.a(new_n305), .b(new_n271), .c(new_n269), .out0(new_n306));
  oao003aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  nano22aa1n02x4               g213(.a(new_n306), .b(new_n307), .c(new_n308), .out0(new_n309));
  oai012aa1n02x5               g214(.a(new_n305), .b(new_n279), .c(new_n287), .o1(new_n310));
  aoi012aa1n02x5               g215(.a(new_n308), .b(new_n310), .c(new_n307), .o1(new_n311));
  norp02aa1n02x5               g216(.a(new_n311), .b(new_n309), .o1(\s[31] ));
  xnrb03aa1n02x5               g217(.a(new_n104), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g218(.a(\a[3] ), .b(\b[2] ), .c(new_n104), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g220(.a(new_n106), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g221(.a(new_n118), .b(new_n120), .c(new_n106), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(new_n119), .out0(\s[6] ));
  oaoi03aa1n02x5               g223(.a(\a[6] ), .b(\b[5] ), .c(new_n317), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g225(.a(new_n109), .b(new_n319), .c(new_n110), .o1(new_n321));
  xnrb03aa1n02x5               g226(.a(new_n321), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g227(.a(new_n263), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


