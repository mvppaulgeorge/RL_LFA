// Benchmark "adder" written by ABC on Thu Jul 11 12:25:10 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n311,
    new_n313, new_n315, new_n317;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[9] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[8] ), .clkout(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  and002aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o(new_n100));
  oaoi03aa1n02x5               g005(.a(\a[2] ), .b(\b[1] ), .c(new_n100), .o1(new_n101));
  xorc02aa1n02x5               g006(.a(\a[4] ), .b(\b[3] ), .out0(new_n102));
  norp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  norb02aa1n02x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  nanp03aa1n02x5               g010(.a(new_n101), .b(new_n102), .c(new_n105), .o1(new_n106));
  160nm_ficinv00aa1n08x5       g011(.clk(new_n103), .clkout(new_n107));
  oao003aa1n02x5               g012(.a(\a[4] ), .b(\b[3] ), .c(new_n107), .carry(new_n108));
  norp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nano23aa1n02x4               g017(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n113));
  norp02aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nano23aa1n02x4               g022(.a(new_n114), .b(new_n116), .c(new_n117), .d(new_n115), .out0(new_n118));
  nanp02aa1n02x5               g023(.a(new_n118), .b(new_n113), .o1(new_n119));
  oa0012aa1n02x5               g024(.a(new_n115), .b(new_n116), .c(new_n114), .o(new_n120));
  oa0012aa1n02x5               g025(.a(new_n110), .b(new_n111), .c(new_n109), .o(new_n121));
  aoi012aa1n02x5               g026(.a(new_n121), .b(new_n113), .c(new_n120), .o1(new_n122));
  aoai13aa1n02x5               g027(.a(new_n122), .b(new_n119), .c(new_n106), .d(new_n108), .o1(new_n123));
  xorc02aa1n02x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  nanp02aa1n02x5               g029(.a(new_n123), .b(new_n124), .o1(new_n125));
  norp02aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nanb02aa1n02x5               g032(.a(new_n126), .b(new_n127), .out0(new_n128));
  xobna2aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n99), .out0(\s[10] ));
  norb02aa1n02x5               g034(.a(new_n124), .b(new_n128), .out0(new_n130));
  oaoi03aa1n02x5               g035(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n131));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n131), .c(new_n123), .d(new_n130), .o1(new_n135));
  aoi112aa1n02x5               g040(.a(new_n134), .b(new_n131), .c(new_n123), .d(new_n130), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g042(.clk(new_n132), .clkout(new_n138));
  norp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanb02aa1n02x5               g045(.a(new_n139), .b(new_n140), .out0(new_n141));
  xobna2aa1n03x5               g046(.a(new_n141), .b(new_n135), .c(new_n138), .out0(\s[12] ));
  aoai13aa1n02x5               g047(.a(new_n127), .b(new_n126), .c(new_n97), .d(new_n98), .o1(new_n143));
  nona23aa1n02x4               g048(.a(new_n140), .b(new_n133), .c(new_n132), .d(new_n139), .out0(new_n144));
  oaoi03aa1n02x5               g049(.a(\a[12] ), .b(\b[11] ), .c(new_n138), .o1(new_n145));
  oabi12aa1n02x5               g050(.a(new_n145), .b(new_n144), .c(new_n143), .out0(new_n146));
  nona23aa1n02x4               g051(.a(new_n134), .b(new_n124), .c(new_n141), .d(new_n128), .out0(new_n147));
  160nm_ficinv00aa1n08x5       g052(.clk(new_n147), .clkout(new_n148));
  xnrc02aa1n02x5               g053(.a(\b[12] ), .b(\a[13] ), .out0(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n149), .clkout(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n146), .c(new_n123), .d(new_n148), .o1(new_n151));
  aoi112aa1n02x5               g056(.a(new_n146), .b(new_n150), .c(new_n123), .d(new_n148), .o1(new_n152));
  norb02aa1n02x5               g057(.a(new_n151), .b(new_n152), .out0(\s[13] ));
  norp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  160nm_ficinv00aa1n08x5       g059(.clk(new_n154), .clkout(new_n155));
  xnrc02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .out0(new_n156));
  xobna2aa1n03x5               g061(.a(new_n156), .b(new_n151), .c(new_n155), .out0(\s[14] ));
  nona32aa1n02x4               g062(.a(new_n123), .b(new_n156), .c(new_n149), .d(new_n147), .out0(new_n158));
  norp02aa1n02x5               g063(.a(new_n156), .b(new_n149), .o1(new_n159));
  oaoi03aa1n02x5               g064(.a(\a[14] ), .b(\b[13] ), .c(new_n155), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n160), .b(new_n146), .c(new_n159), .o1(new_n161));
  xnrc02aa1n02x5               g066(.a(\b[14] ), .b(\a[15] ), .out0(new_n162));
  xobna2aa1n03x5               g067(.a(new_n162), .b(new_n158), .c(new_n161), .out0(\s[15] ));
  norp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  160nm_fiao0012aa1n02p5x5     g069(.a(new_n162), .b(new_n158), .c(new_n161), .o(new_n165));
  xnrc02aa1n02x5               g070(.a(\b[15] ), .b(\a[16] ), .out0(new_n166));
  oaib12aa1n02x5               g071(.a(new_n166), .b(new_n164), .c(new_n165), .out0(new_n167));
  nona22aa1n02x4               g072(.a(new_n165), .b(new_n166), .c(new_n164), .out0(new_n168));
  nanp02aa1n02x5               g073(.a(new_n167), .b(new_n168), .o1(\s[16] ));
  160nm_ficinv00aa1n08x5       g074(.clk(\a[17] ), .clkout(new_n170));
  norp02aa1n02x5               g075(.a(new_n166), .b(new_n162), .o1(new_n171));
  nano22aa1n02x4               g076(.a(new_n147), .b(new_n159), .c(new_n171), .out0(new_n172));
  160nm_ficinv00aa1n08x5       g077(.clk(\a[16] ), .clkout(new_n173));
  160nm_ficinv00aa1n08x5       g078(.clk(\b[15] ), .clkout(new_n174));
  oao003aa1n02x5               g079(.a(new_n173), .b(new_n174), .c(new_n164), .carry(new_n175));
  nano23aa1n02x4               g080(.a(new_n132), .b(new_n139), .c(new_n140), .d(new_n133), .out0(new_n176));
  aoai13aa1n02x5               g081(.a(new_n159), .b(new_n145), .c(new_n176), .d(new_n131), .o1(new_n177));
  160nm_ficinv00aa1n08x5       g082(.clk(new_n160), .clkout(new_n178));
  160nm_ficinv00aa1n08x5       g083(.clk(new_n171), .clkout(new_n179));
  aoi012aa1n02x5               g084(.a(new_n179), .b(new_n177), .c(new_n178), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(new_n180), .b(new_n175), .c(new_n123), .d(new_n172), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(new_n170), .out0(\s[17] ));
  oaoi03aa1n02x5               g087(.a(\a[17] ), .b(\b[16] ), .c(new_n181), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[18] ), .clkout(new_n185));
  xroi22aa1d04x5               g090(.a(new_n170), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(new_n186), .clkout(new_n187));
  oai022aa1n02x5               g092(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n188));
  oaib12aa1n02x5               g093(.a(new_n188), .b(new_n185), .c(\b[17] ), .out0(new_n189));
  oai012aa1n02x5               g094(.a(new_n189), .b(new_n181), .c(new_n187), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g096(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nanb02aa1n02x5               g099(.a(new_n193), .b(new_n194), .out0(new_n195));
  160nm_ficinv00aa1n08x5       g100(.clk(new_n195), .clkout(new_n196));
  norp02aa1n02x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  nanb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(new_n199));
  aoai13aa1n02x5               g104(.a(new_n199), .b(new_n193), .c(new_n190), .d(new_n196), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(new_n123), .b(new_n172), .o1(new_n201));
  160nm_ficinv00aa1n08x5       g106(.clk(new_n175), .clkout(new_n202));
  oai112aa1n02x5               g107(.a(new_n201), .b(new_n202), .c(new_n179), .d(new_n161), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(\b[16] ), .b(new_n170), .out0(new_n204));
  oaoi03aa1n02x5               g109(.a(\a[18] ), .b(\b[17] ), .c(new_n204), .o1(new_n205));
  aoai13aa1n02x5               g110(.a(new_n196), .b(new_n205), .c(new_n203), .d(new_n186), .o1(new_n206));
  nona22aa1n02x4               g111(.a(new_n206), .b(new_n199), .c(new_n193), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n200), .b(new_n207), .o1(\s[20] ));
  nona23aa1n02x4               g113(.a(new_n198), .b(new_n194), .c(new_n193), .d(new_n197), .out0(new_n209));
  oa0012aa1n02x5               g114(.a(new_n198), .b(new_n197), .c(new_n193), .o(new_n210));
  160nm_ficinv00aa1n08x5       g115(.clk(new_n210), .clkout(new_n211));
  oai012aa1n02x5               g116(.a(new_n211), .b(new_n209), .c(new_n189), .o1(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  nano23aa1n02x4               g118(.a(new_n193), .b(new_n197), .c(new_n198), .d(new_n194), .out0(new_n214));
  nanp02aa1n02x5               g119(.a(new_n186), .b(new_n214), .o1(new_n215));
  oai012aa1n02x5               g120(.a(new_n213), .b(new_n181), .c(new_n215), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  xnrc02aa1n02x5               g123(.a(\b[20] ), .b(\a[21] ), .out0(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(new_n219), .clkout(new_n220));
  xnrc02aa1n02x5               g125(.a(\b[21] ), .b(\a[22] ), .out0(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n218), .c(new_n216), .d(new_n220), .o1(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(new_n215), .clkout(new_n223));
  aoai13aa1n02x5               g128(.a(new_n220), .b(new_n212), .c(new_n203), .d(new_n223), .o1(new_n224));
  nona22aa1n02x4               g129(.a(new_n224), .b(new_n221), .c(new_n218), .out0(new_n225));
  nanp02aa1n02x5               g130(.a(new_n222), .b(new_n225), .o1(\s[22] ));
  norp02aa1n02x5               g131(.a(new_n221), .b(new_n219), .o1(new_n227));
  nano22aa1n02x4               g132(.a(new_n187), .b(new_n227), .c(new_n214), .out0(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(new_n228), .clkout(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(\a[22] ), .clkout(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(\b[21] ), .clkout(new_n231));
  oao003aa1n02x5               g136(.a(new_n230), .b(new_n231), .c(new_n218), .carry(new_n232));
  aoi012aa1n02x5               g137(.a(new_n232), .b(new_n212), .c(new_n227), .o1(new_n233));
  oai012aa1n02x5               g138(.a(new_n233), .b(new_n181), .c(new_n229), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  xorc02aa1n02x5               g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  xnrc02aa1n02x5               g142(.a(\b[23] ), .b(\a[24] ), .out0(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n236), .c(new_n234), .d(new_n237), .o1(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n233), .clkout(new_n240));
  aoai13aa1n02x5               g145(.a(new_n237), .b(new_n240), .c(new_n203), .d(new_n228), .o1(new_n241));
  nona22aa1n02x4               g146(.a(new_n241), .b(new_n238), .c(new_n236), .out0(new_n242));
  nanp02aa1n02x5               g147(.a(new_n239), .b(new_n242), .o1(\s[24] ));
  norb02aa1n02x5               g148(.a(new_n237), .b(new_n238), .out0(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n244), .clkout(new_n245));
  nano32aa1n02x4               g150(.a(new_n245), .b(new_n186), .c(new_n227), .d(new_n214), .out0(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n246), .clkout(new_n247));
  aoai13aa1n02x5               g152(.a(new_n227), .b(new_n210), .c(new_n214), .d(new_n205), .o1(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n232), .clkout(new_n249));
  oai022aa1n02x5               g154(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n250));
  aob012aa1n02x5               g155(.a(new_n250), .b(\b[23] ), .c(\a[24] ), .out0(new_n251));
  aoai13aa1n02x5               g156(.a(new_n251), .b(new_n245), .c(new_n248), .d(new_n249), .o1(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n252), .clkout(new_n253));
  oai012aa1n02x5               g158(.a(new_n253), .b(new_n181), .c(new_n247), .o1(new_n254));
  xorb03aa1n02x5               g159(.a(new_n254), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  xorc02aa1n02x5               g161(.a(\a[25] ), .b(\b[24] ), .out0(new_n257));
  xnrc02aa1n02x5               g162(.a(\b[25] ), .b(\a[26] ), .out0(new_n258));
  aoai13aa1n02x5               g163(.a(new_n258), .b(new_n256), .c(new_n254), .d(new_n257), .o1(new_n259));
  aoai13aa1n02x5               g164(.a(new_n257), .b(new_n252), .c(new_n203), .d(new_n246), .o1(new_n260));
  nona22aa1n02x4               g165(.a(new_n260), .b(new_n258), .c(new_n256), .out0(new_n261));
  nanp02aa1n02x5               g166(.a(new_n259), .b(new_n261), .o1(\s[26] ));
  norb02aa1n02x5               g167(.a(new_n257), .b(new_n258), .out0(new_n263));
  nano23aa1n02x4               g168(.a(new_n215), .b(new_n245), .c(new_n263), .d(new_n227), .out0(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n264), .clkout(new_n265));
  nanp02aa1n02x5               g170(.a(\b[25] ), .b(\a[26] ), .o1(new_n266));
  oai022aa1n02x5               g171(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n267));
  aoi022aa1n02x5               g172(.a(new_n252), .b(new_n263), .c(new_n266), .d(new_n267), .o1(new_n268));
  oai012aa1n02x5               g173(.a(new_n268), .b(new_n181), .c(new_n265), .o1(new_n269));
  xorb03aa1n02x5               g174(.a(new_n269), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g175(.a(\b[26] ), .b(\a[27] ), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[27] ), .b(\b[26] ), .out0(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[27] ), .b(\a[28] ), .out0(new_n273));
  aoai13aa1n02x5               g178(.a(new_n273), .b(new_n271), .c(new_n269), .d(new_n272), .o1(new_n274));
  aoai13aa1n02x5               g179(.a(new_n244), .b(new_n232), .c(new_n212), .d(new_n227), .o1(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n263), .clkout(new_n276));
  nanp02aa1n02x5               g181(.a(new_n267), .b(new_n266), .o1(new_n277));
  aoai13aa1n02x5               g182(.a(new_n277), .b(new_n276), .c(new_n275), .d(new_n251), .o1(new_n278));
  aoai13aa1n02x5               g183(.a(new_n272), .b(new_n278), .c(new_n203), .d(new_n264), .o1(new_n279));
  nona22aa1n02x4               g184(.a(new_n279), .b(new_n273), .c(new_n271), .out0(new_n280));
  nanp02aa1n02x5               g185(.a(new_n274), .b(new_n280), .o1(\s[28] ));
  norb02aa1n02x5               g186(.a(new_n272), .b(new_n273), .out0(new_n282));
  aoai13aa1n02x5               g187(.a(new_n282), .b(new_n278), .c(new_n203), .d(new_n264), .o1(new_n283));
  160nm_ficinv00aa1n08x5       g188(.clk(new_n271), .clkout(new_n284));
  oaoi03aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .o1(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  nona22aa1n02x4               g191(.a(new_n283), .b(new_n285), .c(new_n286), .out0(new_n287));
  aoai13aa1n02x5               g192(.a(new_n286), .b(new_n285), .c(new_n269), .d(new_n282), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(new_n288), .b(new_n287), .o1(\s[29] ));
  xnrb03aa1n02x5               g194(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g195(.a(new_n272), .b(new_n286), .c(new_n273), .out0(new_n291));
  oao003aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .carry(new_n292));
  oaoi03aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .o1(new_n293));
  xorc02aa1n02x5               g198(.a(\a[30] ), .b(\b[29] ), .out0(new_n294));
  160nm_ficinv00aa1n08x5       g199(.clk(new_n294), .clkout(new_n295));
  aoai13aa1n02x5               g200(.a(new_n295), .b(new_n293), .c(new_n269), .d(new_n291), .o1(new_n296));
  aoai13aa1n02x5               g201(.a(new_n291), .b(new_n278), .c(new_n203), .d(new_n264), .o1(new_n297));
  nona22aa1n02x4               g202(.a(new_n297), .b(new_n293), .c(new_n295), .out0(new_n298));
  nanp02aa1n02x5               g203(.a(new_n296), .b(new_n298), .o1(\s[30] ));
  nano23aa1n02x4               g204(.a(new_n286), .b(new_n273), .c(new_n294), .d(new_n272), .out0(new_n300));
  aoai13aa1n02x5               g205(.a(new_n300), .b(new_n278), .c(new_n203), .d(new_n264), .o1(new_n301));
  nanp02aa1n02x5               g206(.a(new_n293), .b(new_n294), .o1(new_n302));
  oai012aa1n02x5               g207(.a(new_n302), .b(\b[29] ), .c(\a[30] ), .o1(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  nona22aa1n02x4               g209(.a(new_n301), .b(new_n303), .c(new_n304), .out0(new_n305));
  aoai13aa1n02x5               g210(.a(new_n304), .b(new_n303), .c(new_n269), .d(new_n300), .o1(new_n306));
  nanp02aa1n02x5               g211(.a(new_n306), .b(new_n305), .o1(\s[31] ));
  xobna2aa1n03x5               g212(.a(new_n101), .b(new_n104), .c(new_n107), .out0(\s[3] ));
  nanp02aa1n02x5               g213(.a(new_n101), .b(new_n105), .o1(new_n309));
  xnbna2aa1n03x5               g214(.a(new_n102), .b(new_n309), .c(new_n107), .out0(\s[4] ));
  nanp02aa1n02x5               g215(.a(new_n106), .b(new_n108), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g217(.a(new_n116), .b(new_n311), .c(new_n117), .o1(new_n313));
  xnrb03aa1n02x5               g218(.a(new_n313), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  160nm_fiao0012aa1n02p5x5     g219(.a(new_n120), .b(new_n311), .c(new_n118), .o(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g221(.a(new_n111), .b(new_n315), .c(new_n112), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g223(.a(new_n123), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


