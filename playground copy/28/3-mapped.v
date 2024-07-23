// Benchmark "adder" written by ABC on Thu Jul 11 12:55:31 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n307, new_n309, new_n311, new_n313,
    new_n315;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .o(new_n97));
  and002aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(new_n98), .clkout(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\a[9] ), .clkout(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\b[8] ), .clkout(new_n101));
  norp02aa1n02x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nano23aa1n02x4               g010(.a(new_n102), .b(new_n104), .c(new_n105), .d(new_n103), .out0(new_n106));
  nanp02aa1n02x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  oai012aa1n02x5               g014(.a(new_n107), .b(new_n109), .c(new_n108), .o1(new_n110));
  oa0012aa1n02x5               g015(.a(new_n103), .b(new_n104), .c(new_n102), .o(new_n111));
  aoib12aa1n02x5               g016(.a(new_n111), .b(new_n106), .c(new_n110), .out0(new_n112));
  nanp02aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n107), .b(new_n113), .c(new_n109), .d(new_n108), .out0(new_n114));
  and002aa1n02x5               g019(.a(\b[3] ), .b(\a[4] ), .o(new_n115));
  oa0022aa1n02x5               g020(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n116));
  160nm_ficinv00aa1n08x5       g021(.clk(\a[3] ), .clkout(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\b[2] ), .clkout(new_n118));
  nanp02aa1n02x5               g023(.a(new_n118), .b(new_n117), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(\b[2] ), .b(\a[3] ), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n119), .b(new_n120), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[1] ), .b(\a[2] ), .o1(new_n122));
  norp02aa1n02x5               g027(.a(\b[1] ), .b(\a[2] ), .o1(new_n123));
  nanp02aa1n02x5               g028(.a(\b[0] ), .b(\a[1] ), .o1(new_n124));
  oai012aa1n02x5               g029(.a(new_n122), .b(new_n123), .c(new_n124), .o1(new_n125));
  oai012aa1n02x5               g030(.a(new_n116), .b(new_n125), .c(new_n121), .o1(new_n126));
  nona23aa1n02x4               g031(.a(new_n126), .b(new_n106), .c(new_n114), .d(new_n115), .out0(new_n127));
  nanp02aa1n02x5               g032(.a(new_n127), .b(new_n112), .o1(new_n128));
  oaoi03aa1n02x5               g033(.a(new_n100), .b(new_n101), .c(new_n128), .o1(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n97), .c(new_n99), .out0(\s[10] ));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  160nm_ficinv00aa1n08x5       g038(.clk(new_n133), .clkout(new_n134));
  nanp02aa1n02x5               g039(.a(new_n129), .b(new_n97), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n134), .b(new_n135), .c(new_n99), .out0(\s[11] ));
  aoi112aa1n02x5               g041(.a(new_n134), .b(new_n98), .c(new_n129), .d(new_n97), .o1(new_n137));
  norp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(new_n140));
  oai012aa1n02x5               g045(.a(new_n140), .b(new_n137), .c(new_n131), .o1(new_n141));
  nona22aa1n02x4               g046(.a(new_n135), .b(new_n134), .c(new_n98), .out0(new_n142));
  nona22aa1n02x4               g047(.a(new_n142), .b(new_n140), .c(new_n131), .out0(new_n143));
  nanp02aa1n02x5               g048(.a(new_n141), .b(new_n143), .o1(\s[12] ));
  nano23aa1n02x4               g049(.a(new_n131), .b(new_n138), .c(new_n139), .d(new_n132), .out0(new_n145));
  oaoi13aa1n02x5               g050(.a(new_n98), .b(new_n97), .c(\a[9] ), .d(\b[8] ), .o1(new_n146));
  oa0012aa1n02x5               g051(.a(new_n139), .b(new_n138), .c(new_n131), .o(new_n147));
  aoi012aa1n02x5               g052(.a(new_n147), .b(new_n145), .c(new_n146), .o1(new_n148));
  xorc02aa1n02x5               g053(.a(\a[10] ), .b(\b[9] ), .out0(new_n149));
  xnrc02aa1n02x5               g054(.a(\b[8] ), .b(\a[9] ), .out0(new_n150));
  nanb03aa1n02x5               g055(.a(new_n150), .b(new_n145), .c(new_n149), .out0(new_n151));
  aoai13aa1n02x5               g056(.a(new_n148), .b(new_n151), .c(new_n127), .d(new_n112), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g058(.clk(\a[14] ), .clkout(new_n154));
  norp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  xnrc02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .out0(new_n156));
  aoib12aa1n02x5               g061(.a(new_n155), .b(new_n152), .c(new_n156), .out0(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(new_n154), .out0(\s[14] ));
  xnrc02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .out0(new_n159));
  norp02aa1n02x5               g064(.a(new_n159), .b(new_n156), .o1(new_n160));
  160nm_ficinv00aa1n08x5       g065(.clk(\b[13] ), .clkout(new_n161));
  oao003aa1n02x5               g066(.a(new_n154), .b(new_n161), .c(new_n155), .carry(new_n162));
  160nm_fiao0012aa1n02p5x5     g067(.a(new_n162), .b(new_n152), .c(new_n160), .o(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  xnrc02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .out0(new_n166));
  160nm_ficinv00aa1n08x5       g071(.clk(new_n166), .clkout(new_n167));
  norp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n165), .c(new_n163), .d(new_n167), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n167), .b(new_n162), .c(new_n152), .d(new_n160), .o1(new_n172));
  nona22aa1n02x4               g077(.a(new_n172), .b(new_n170), .c(new_n165), .out0(new_n173));
  nanp02aa1n02x5               g078(.a(new_n171), .b(new_n173), .o1(\s[16] ));
  norp02aa1n02x5               g079(.a(new_n166), .b(new_n170), .o1(new_n175));
  oai022aa1n02x5               g080(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n176));
  aoi022aa1n02x5               g081(.a(new_n175), .b(new_n162), .c(new_n169), .d(new_n176), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(new_n160), .b(new_n175), .o1(new_n178));
  oai012aa1n02x5               g083(.a(new_n177), .b(new_n148), .c(new_n178), .o1(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(new_n179), .clkout(new_n180));
  nano22aa1n02x4               g085(.a(new_n151), .b(new_n160), .c(new_n175), .out0(new_n181));
  nanp02aa1n02x5               g086(.a(new_n128), .b(new_n181), .o1(new_n182));
  xorc02aa1n02x5               g087(.a(\a[17] ), .b(\b[16] ), .out0(new_n183));
  xnbna2aa1n03x5               g088(.a(new_n183), .b(new_n182), .c(new_n180), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[17] ), .clkout(new_n185));
  nanb02aa1n02x5               g090(.a(\b[16] ), .b(new_n185), .out0(new_n186));
  aoai13aa1n02x5               g091(.a(new_n183), .b(new_n179), .c(new_n128), .d(new_n181), .o1(new_n187));
  xnrc02aa1n02x5               g092(.a(\b[17] ), .b(\a[18] ), .out0(new_n188));
  xobna2aa1n03x5               g093(.a(new_n188), .b(new_n187), .c(new_n186), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g094(.clk(\a[18] ), .clkout(new_n190));
  xroi22aa1d04x5               g095(.a(new_n185), .b(\b[16] ), .c(new_n190), .d(\b[17] ), .out0(new_n191));
  aoai13aa1n02x5               g096(.a(new_n191), .b(new_n179), .c(new_n128), .d(new_n181), .o1(new_n192));
  oai022aa1n02x5               g097(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n193));
  oaib12aa1n02x5               g098(.a(new_n193), .b(new_n190), .c(\b[17] ), .out0(new_n194));
  norp02aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nanb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(new_n197));
  160nm_ficinv00aa1n08x5       g102(.clk(new_n197), .clkout(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n192), .c(new_n194), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g105(.a(new_n192), .b(new_n194), .o1(new_n201));
  norp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  aoai13aa1n02x5               g109(.a(new_n204), .b(new_n195), .c(new_n201), .d(new_n198), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(new_n201), .b(new_n198), .o1(new_n206));
  nona22aa1n02x4               g111(.a(new_n206), .b(new_n204), .c(new_n195), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n207), .b(new_n205), .o1(\s[20] ));
  nona23aa1n02x4               g113(.a(new_n203), .b(new_n196), .c(new_n195), .d(new_n202), .out0(new_n209));
  oai012aa1n02x5               g114(.a(new_n203), .b(new_n202), .c(new_n195), .o1(new_n210));
  oai012aa1n02x5               g115(.a(new_n210), .b(new_n209), .c(new_n194), .o1(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(new_n211), .clkout(new_n212));
  nano23aa1n02x4               g117(.a(new_n195), .b(new_n202), .c(new_n203), .d(new_n196), .out0(new_n213));
  nanb03aa1n02x5               g118(.a(new_n188), .b(new_n213), .c(new_n183), .out0(new_n214));
  aoai13aa1n02x5               g119(.a(new_n212), .b(new_n214), .c(new_n182), .d(new_n180), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xnrc02aa1n02x5               g122(.a(\b[20] ), .b(\a[21] ), .out0(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n218), .clkout(new_n219));
  xnrc02aa1n02x5               g124(.a(\b[21] ), .b(\a[22] ), .out0(new_n220));
  aoai13aa1n02x5               g125(.a(new_n220), .b(new_n217), .c(new_n215), .d(new_n219), .o1(new_n221));
  nanp02aa1n02x5               g126(.a(new_n215), .b(new_n219), .o1(new_n222));
  nona22aa1n02x4               g127(.a(new_n222), .b(new_n220), .c(new_n217), .out0(new_n223));
  nanp02aa1n02x5               g128(.a(new_n223), .b(new_n221), .o1(\s[22] ));
  norp02aa1n02x5               g129(.a(new_n220), .b(new_n218), .o1(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(\a[22] ), .clkout(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(\b[21] ), .clkout(new_n227));
  oao003aa1n02x5               g132(.a(new_n226), .b(new_n227), .c(new_n217), .carry(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n211), .c(new_n225), .o1(new_n229));
  nanp03aa1n02x5               g134(.a(new_n191), .b(new_n225), .c(new_n213), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n229), .b(new_n230), .c(new_n182), .d(new_n180), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  xnrc02aa1n02x5               g139(.a(\b[23] ), .b(\a[24] ), .out0(new_n235));
  aoai13aa1n02x5               g140(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n236));
  nanp02aa1n02x5               g141(.a(new_n231), .b(new_n234), .o1(new_n237));
  nona22aa1n02x4               g142(.a(new_n237), .b(new_n235), .c(new_n233), .out0(new_n238));
  nanp02aa1n02x5               g143(.a(new_n238), .b(new_n236), .o1(\s[24] ));
  norb02aa1n02x5               g144(.a(new_n234), .b(new_n235), .out0(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n240), .clkout(new_n241));
  nano32aa1n02x4               g146(.a(new_n241), .b(new_n191), .c(new_n225), .d(new_n213), .out0(new_n242));
  aoai13aa1n02x5               g147(.a(new_n242), .b(new_n179), .c(new_n128), .d(new_n181), .o1(new_n243));
  oaoi03aa1n02x5               g148(.a(\a[18] ), .b(\b[17] ), .c(new_n186), .o1(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n210), .clkout(new_n245));
  aoai13aa1n02x5               g150(.a(new_n225), .b(new_n245), .c(new_n213), .d(new_n244), .o1(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n228), .clkout(new_n247));
  oai022aa1n02x5               g152(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n248));
  aob012aa1n02x5               g153(.a(new_n248), .b(\b[23] ), .c(\a[24] ), .out0(new_n249));
  aoai13aa1n02x5               g154(.a(new_n249), .b(new_n241), .c(new_n246), .d(new_n247), .o1(new_n250));
  nanb02aa1n02x5               g155(.a(new_n250), .b(new_n243), .out0(new_n251));
  xorb03aa1n02x5               g156(.a(new_n251), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  norp02aa1n02x5               g159(.a(\b[25] ), .b(\a[26] ), .o1(new_n255));
  nanp02aa1n02x5               g160(.a(\b[25] ), .b(\a[26] ), .o1(new_n256));
  nanb02aa1n02x5               g161(.a(new_n255), .b(new_n256), .out0(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n253), .c(new_n251), .d(new_n254), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(new_n182), .b(new_n180), .o1(new_n259));
  aoai13aa1n02x5               g164(.a(new_n254), .b(new_n250), .c(new_n259), .d(new_n242), .o1(new_n260));
  nona22aa1n02x4               g165(.a(new_n260), .b(new_n257), .c(new_n253), .out0(new_n261));
  nanp02aa1n02x5               g166(.a(new_n258), .b(new_n261), .o1(\s[26] ));
  norb02aa1n02x5               g167(.a(new_n254), .b(new_n257), .out0(new_n263));
  nanp02aa1n02x5               g168(.a(new_n250), .b(new_n263), .o1(new_n264));
  oai012aa1n02x5               g169(.a(new_n256), .b(new_n255), .c(new_n253), .o1(new_n265));
  nano22aa1n02x4               g170(.a(new_n230), .b(new_n240), .c(new_n263), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n266), .b(new_n179), .c(new_n128), .d(new_n181), .o1(new_n267));
  nanp03aa1n02x5               g172(.a(new_n267), .b(new_n264), .c(new_n265), .o1(new_n268));
  xorb03aa1n02x5               g173(.a(new_n268), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  xorc02aa1n02x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[27] ), .b(\a[28] ), .out0(new_n272));
  aoai13aa1n02x5               g177(.a(new_n272), .b(new_n270), .c(new_n268), .d(new_n271), .o1(new_n273));
  aoai13aa1n02x5               g178(.a(new_n240), .b(new_n228), .c(new_n211), .d(new_n225), .o1(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n263), .clkout(new_n275));
  aoai13aa1n02x5               g180(.a(new_n265), .b(new_n275), .c(new_n274), .d(new_n249), .o1(new_n276));
  aoai13aa1n02x5               g181(.a(new_n271), .b(new_n276), .c(new_n259), .d(new_n266), .o1(new_n277));
  nona22aa1n02x4               g182(.a(new_n277), .b(new_n272), .c(new_n270), .out0(new_n278));
  nanp02aa1n02x5               g183(.a(new_n273), .b(new_n278), .o1(\s[28] ));
  norb02aa1n02x5               g184(.a(new_n271), .b(new_n272), .out0(new_n280));
  aoai13aa1n02x5               g185(.a(new_n280), .b(new_n276), .c(new_n259), .d(new_n266), .o1(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n270), .clkout(new_n282));
  oaoi03aa1n02x5               g187(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .o1(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[28] ), .b(\a[29] ), .out0(new_n284));
  nona22aa1n02x4               g189(.a(new_n281), .b(new_n283), .c(new_n284), .out0(new_n285));
  aoai13aa1n02x5               g190(.a(new_n284), .b(new_n283), .c(new_n268), .d(new_n280), .o1(new_n286));
  nanp02aa1n02x5               g191(.a(new_n286), .b(new_n285), .o1(\s[29] ));
  xorb03aa1n02x5               g192(.a(new_n124), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g193(.a(new_n271), .b(new_n284), .c(new_n272), .out0(new_n289));
  oao003aa1n02x5               g194(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n290));
  oaoi03aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .o1(new_n291));
  xorc02aa1n02x5               g196(.a(\a[30] ), .b(\b[29] ), .out0(new_n292));
  160nm_ficinv00aa1n08x5       g197(.clk(new_n292), .clkout(new_n293));
  aoai13aa1n02x5               g198(.a(new_n293), .b(new_n291), .c(new_n268), .d(new_n289), .o1(new_n294));
  aoai13aa1n02x5               g199(.a(new_n289), .b(new_n276), .c(new_n259), .d(new_n266), .o1(new_n295));
  nona22aa1n02x4               g200(.a(new_n295), .b(new_n291), .c(new_n293), .out0(new_n296));
  nanp02aa1n02x5               g201(.a(new_n294), .b(new_n296), .o1(\s[30] ));
  nanp02aa1n02x5               g202(.a(new_n291), .b(new_n292), .o1(new_n298));
  oai012aa1n02x5               g203(.a(new_n298), .b(\b[29] ), .c(\a[30] ), .o1(new_n299));
  nano23aa1n02x4               g204(.a(new_n284), .b(new_n272), .c(new_n292), .d(new_n271), .out0(new_n300));
  aoai13aa1n02x5               g205(.a(new_n300), .b(new_n276), .c(new_n259), .d(new_n266), .o1(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  nona22aa1n02x4               g207(.a(new_n301), .b(new_n302), .c(new_n299), .out0(new_n303));
  aoai13aa1n02x5               g208(.a(new_n302), .b(new_n299), .c(new_n268), .d(new_n300), .o1(new_n304));
  nanp02aa1n02x5               g209(.a(new_n304), .b(new_n303), .o1(\s[31] ));
  xnbna2aa1n03x5               g210(.a(new_n125), .b(new_n119), .c(new_n120), .out0(\s[3] ));
  oaoi03aa1n02x5               g211(.a(\a[3] ), .b(\b[2] ), .c(new_n125), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  oaoi13aa1n02x5               g213(.a(new_n115), .b(new_n116), .c(new_n125), .d(new_n121), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g215(.a(new_n109), .b(new_n309), .c(new_n113), .o1(new_n311));
  xnrb03aa1n02x5               g216(.a(new_n311), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g217(.a(new_n110), .b(new_n114), .c(new_n309), .out0(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g219(.a(new_n104), .b(new_n313), .c(new_n105), .o1(new_n315));
  xnrb03aa1n02x5               g220(.a(new_n315), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g221(.a(new_n150), .b(new_n127), .c(new_n112), .out0(\s[9] ));
endmodule


