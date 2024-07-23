// Benchmark "adder" written by ABC on Thu Jul 11 11:15:13 2024

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
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n307, new_n310, new_n312, new_n313,
    new_n315;
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
  nanp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[0] ), .b(\a[1] ), .o1(new_n110));
  oai012aa1n02x5               g015(.a(new_n108), .b(new_n109), .c(new_n110), .o1(new_n111));
  160nm_ficinv00aa1n08x5       g016(.clk(\b[3] ), .clkout(new_n112));
  aboi22aa1n03x5               g017(.a(\a[4] ), .b(new_n112), .c(new_n103), .d(new_n104), .out0(new_n113));
  oaoi13aa1n02x5               g018(.a(new_n102), .b(new_n113), .c(new_n111), .d(new_n107), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nona23aa1n02x4               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  xnrc02aa1n02x5               g024(.a(\b[5] ), .b(\a[6] ), .out0(new_n120));
  xnrc02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .out0(new_n121));
  norp03aa1n02x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(new_n114), .b(new_n122), .o1(new_n123));
  orn002aa1n02x5               g028(.a(\a[5] ), .b(\b[4] ), .o(new_n124));
  oaoi03aa1n02x5               g029(.a(\a[6] ), .b(\b[5] ), .c(new_n124), .o1(new_n125));
  oai012aa1n02x5               g030(.a(new_n116), .b(new_n117), .c(new_n115), .o1(new_n126));
  oaib12aa1n02x5               g031(.a(new_n126), .b(new_n119), .c(new_n125), .out0(new_n127));
  xnrc02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .out0(new_n128));
  nona22aa1n02x4               g033(.a(new_n123), .b(new_n127), .c(new_n128), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n101), .out0(\s[10] ));
  orn002aa1n02x5               g035(.a(\a[11] ), .b(\b[10] ), .o(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  aoai13aa1n02x5               g037(.a(new_n98), .b(new_n97), .c(new_n129), .d(new_n101), .o1(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n131), .c(new_n132), .out0(\s[11] ));
  nanp02aa1n02x5               g039(.a(new_n131), .b(new_n132), .o1(new_n135));
  xnrc02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .out0(new_n136));
  oai112aa1n02x5               g041(.a(new_n131), .b(new_n136), .c(new_n133), .d(new_n135), .o1(new_n137));
  oaoi13aa1n02x5               g042(.a(new_n136), .b(new_n131), .c(new_n133), .d(new_n135), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(\s[12] ));
  nano23aa1n02x4               g044(.a(new_n115), .b(new_n117), .c(new_n118), .d(new_n116), .out0(new_n140));
  aobi12aa1n02x5               g045(.a(new_n126), .b(new_n140), .c(new_n125), .out0(new_n141));
  xorc02aa1n02x5               g046(.a(\a[12] ), .b(\b[11] ), .out0(new_n142));
  norp02aa1n02x5               g047(.a(\b[8] ), .b(\a[9] ), .o1(new_n143));
  norp03aa1n02x5               g048(.a(new_n100), .b(new_n143), .c(new_n97), .o1(new_n144));
  nanp03aa1n02x5               g049(.a(new_n131), .b(new_n98), .c(new_n132), .o1(new_n145));
  nanb03aa1n02x5               g050(.a(new_n145), .b(new_n142), .c(new_n144), .out0(new_n146));
  norp02aa1n02x5               g051(.a(new_n143), .b(new_n97), .o1(new_n147));
  oao003aa1n02x5               g052(.a(\a[12] ), .b(\b[11] ), .c(new_n131), .carry(new_n148));
  oai013aa1n02x4               g053(.a(new_n148), .b(new_n145), .c(new_n136), .d(new_n147), .o1(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n149), .clkout(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n146), .c(new_n123), .d(new_n141), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n153), .b(new_n151), .c(new_n154), .o1(new_n155));
  xnrb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[14] ), .b(\a[15] ), .o1(new_n158));
  nanb02aa1n02x5               g063(.a(new_n157), .b(new_n158), .out0(new_n159));
  160nm_ficinv00aa1n08x5       g064(.clk(new_n159), .clkout(new_n160));
  norp02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nona23aa1n02x4               g067(.a(new_n162), .b(new_n154), .c(new_n153), .d(new_n161), .out0(new_n163));
  160nm_ficinv00aa1n08x5       g068(.clk(new_n163), .clkout(new_n164));
  aoi012aa1n02x5               g069(.a(new_n161), .b(new_n153), .c(new_n162), .o1(new_n165));
  160nm_ficinv00aa1n08x5       g070(.clk(new_n165), .clkout(new_n166));
  aoai13aa1n02x5               g071(.a(new_n160), .b(new_n166), .c(new_n151), .d(new_n164), .o1(new_n167));
  aoi112aa1n02x5               g072(.a(new_n160), .b(new_n166), .c(new_n151), .d(new_n164), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(\s[15] ));
  norp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  oai112aa1n02x5               g077(.a(new_n167), .b(new_n172), .c(\b[14] ), .d(\a[15] ), .o1(new_n173));
  oaoi13aa1n02x5               g078(.a(new_n172), .b(new_n167), .c(\a[15] ), .d(\b[14] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n173), .b(new_n174), .out0(\s[16] ));
  nano23aa1n02x4               g080(.a(new_n157), .b(new_n170), .c(new_n171), .d(new_n158), .out0(new_n176));
  nano22aa1n02x4               g081(.a(new_n146), .b(new_n164), .c(new_n176), .out0(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n127), .c(new_n114), .d(new_n122), .o1(new_n178));
  norp03aa1n02x5               g083(.a(new_n163), .b(new_n172), .c(new_n159), .o1(new_n179));
  norp03aa1n02x5               g084(.a(new_n165), .b(new_n172), .c(new_n159), .o1(new_n180));
  oa0012aa1n02x5               g085(.a(new_n171), .b(new_n170), .c(new_n157), .o(new_n181));
  aoi112aa1n02x5               g086(.a(new_n181), .b(new_n180), .c(new_n149), .d(new_n179), .o1(new_n182));
  nanp02aa1n02x5               g087(.a(new_n178), .b(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[18] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[17] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\b[16] ), .clkout(new_n187));
  oaoi03aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  xroi22aa1d04x5               g094(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(new_n190), .clkout(new_n191));
  norp02aa1n02x5               g096(.a(\b[17] ), .b(\a[18] ), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(\b[17] ), .b(\a[18] ), .o1(new_n193));
  aoi013aa1n02x4               g098(.a(new_n192), .b(new_n193), .c(new_n186), .d(new_n187), .o1(new_n194));
  aoai13aa1n02x5               g099(.a(new_n194), .b(new_n191), .c(new_n178), .d(new_n182), .o1(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g101(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  norp02aa1n02x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  aoi112aa1n02x5               g107(.a(new_n198), .b(new_n202), .c(new_n195), .d(new_n199), .o1(new_n203));
  aoai13aa1n02x5               g108(.a(new_n202), .b(new_n198), .c(new_n195), .d(new_n199), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(\s[20] ));
  nano23aa1n02x4               g110(.a(new_n198), .b(new_n200), .c(new_n201), .d(new_n199), .out0(new_n206));
  nanp02aa1n02x5               g111(.a(new_n190), .b(new_n206), .o1(new_n207));
  nona23aa1n02x4               g112(.a(new_n201), .b(new_n199), .c(new_n198), .d(new_n200), .out0(new_n208));
  oai012aa1n02x5               g113(.a(new_n201), .b(new_n200), .c(new_n198), .o1(new_n209));
  oai012aa1n02x5               g114(.a(new_n209), .b(new_n208), .c(new_n194), .o1(new_n210));
  160nm_ficinv00aa1n08x5       g115(.clk(new_n210), .clkout(new_n211));
  aoai13aa1n02x5               g116(.a(new_n211), .b(new_n207), .c(new_n178), .d(new_n182), .o1(new_n212));
  xorb03aa1n02x5               g117(.a(new_n212), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g118(.a(\b[20] ), .b(\a[21] ), .o1(new_n214));
  xorc02aa1n02x5               g119(.a(\a[21] ), .b(\b[20] ), .out0(new_n215));
  xorc02aa1n02x5               g120(.a(\a[22] ), .b(\b[21] ), .out0(new_n216));
  aoi112aa1n02x5               g121(.a(new_n214), .b(new_n216), .c(new_n212), .d(new_n215), .o1(new_n217));
  aoai13aa1n02x5               g122(.a(new_n216), .b(new_n214), .c(new_n212), .d(new_n215), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n218), .b(new_n217), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g124(.clk(\a[21] ), .clkout(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(\a[22] ), .clkout(new_n221));
  xroi22aa1d04x5               g126(.a(new_n220), .b(\b[20] ), .c(new_n221), .d(\b[21] ), .out0(new_n222));
  nanp03aa1n02x5               g127(.a(new_n222), .b(new_n190), .c(new_n206), .o1(new_n223));
  nona22aa1n02x4               g128(.a(new_n193), .b(\b[16] ), .c(\a[17] ), .out0(new_n224));
  oaib12aa1n02x5               g129(.a(new_n224), .b(\b[17] ), .c(new_n185), .out0(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n209), .clkout(new_n226));
  aoai13aa1n02x5               g131(.a(new_n222), .b(new_n226), .c(new_n206), .d(new_n225), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(\b[21] ), .clkout(new_n228));
  oao003aa1n02x5               g133(.a(new_n221), .b(new_n228), .c(new_n214), .carry(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(new_n229), .clkout(new_n230));
  nanp02aa1n02x5               g135(.a(new_n227), .b(new_n230), .o1(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(new_n231), .clkout(new_n232));
  aoai13aa1n02x5               g137(.a(new_n232), .b(new_n223), .c(new_n178), .d(new_n182), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  xorc02aa1n02x5               g140(.a(\a[23] ), .b(\b[22] ), .out0(new_n236));
  xorc02aa1n02x5               g141(.a(\a[24] ), .b(\b[23] ), .out0(new_n237));
  aoi112aa1n02x5               g142(.a(new_n235), .b(new_n237), .c(new_n233), .d(new_n236), .o1(new_n238));
  aoai13aa1n02x5               g143(.a(new_n237), .b(new_n235), .c(new_n233), .d(new_n236), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(\s[24] ));
  and002aa1n02x5               g145(.a(new_n237), .b(new_n236), .o(new_n241));
  nanb03aa1n02x5               g146(.a(new_n207), .b(new_n241), .c(new_n222), .out0(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n241), .clkout(new_n243));
  nanp02aa1n02x5               g148(.a(\b[23] ), .b(\a[24] ), .o1(new_n244));
  oai022aa1n02x5               g149(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n245));
  nanp02aa1n02x5               g150(.a(new_n245), .b(new_n244), .o1(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n243), .c(new_n227), .d(new_n230), .o1(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n247), .clkout(new_n248));
  aoai13aa1n02x5               g153(.a(new_n248), .b(new_n242), .c(new_n178), .d(new_n182), .o1(new_n249));
  xorb03aa1n02x5               g154(.a(new_n249), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  xorc02aa1n02x5               g156(.a(\a[25] ), .b(\b[24] ), .out0(new_n252));
  xorc02aa1n02x5               g157(.a(\a[26] ), .b(\b[25] ), .out0(new_n253));
  aoi112aa1n02x5               g158(.a(new_n251), .b(new_n253), .c(new_n249), .d(new_n252), .o1(new_n254));
  aoai13aa1n02x5               g159(.a(new_n253), .b(new_n251), .c(new_n249), .d(new_n252), .o1(new_n255));
  norb02aa1n02x5               g160(.a(new_n255), .b(new_n254), .out0(\s[26] ));
  nanp02aa1n02x5               g161(.a(new_n123), .b(new_n141), .o1(new_n257));
  nanp02aa1n02x5               g162(.a(new_n149), .b(new_n179), .o1(new_n258));
  nona22aa1n02x4               g163(.a(new_n258), .b(new_n181), .c(new_n180), .out0(new_n259));
  and002aa1n02x5               g164(.a(new_n253), .b(new_n252), .o(new_n260));
  nano22aa1n02x4               g165(.a(new_n223), .b(new_n241), .c(new_n260), .out0(new_n261));
  aoai13aa1n02x5               g166(.a(new_n261), .b(new_n259), .c(new_n257), .d(new_n177), .o1(new_n262));
  oai022aa1n02x5               g167(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n263));
  aob012aa1n02x5               g168(.a(new_n263), .b(\b[25] ), .c(\a[26] ), .out0(new_n264));
  aobi12aa1n02x5               g169(.a(new_n264), .b(new_n247), .c(new_n260), .out0(new_n265));
  xorc02aa1n02x5               g170(.a(\a[27] ), .b(\b[26] ), .out0(new_n266));
  xnbna2aa1n03x5               g171(.a(new_n266), .b(new_n265), .c(new_n262), .out0(\s[27] ));
  norp02aa1n02x5               g172(.a(\b[26] ), .b(\a[27] ), .o1(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n268), .clkout(new_n269));
  aobi12aa1n02x5               g174(.a(new_n266), .b(new_n265), .c(new_n262), .out0(new_n270));
  xnrc02aa1n02x5               g175(.a(\b[27] ), .b(\a[28] ), .out0(new_n271));
  nano22aa1n02x4               g176(.a(new_n270), .b(new_n269), .c(new_n271), .out0(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n261), .clkout(new_n273));
  aoi012aa1n02x5               g178(.a(new_n273), .b(new_n178), .c(new_n182), .o1(new_n274));
  aoai13aa1n02x5               g179(.a(new_n241), .b(new_n229), .c(new_n210), .d(new_n222), .o1(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n260), .clkout(new_n276));
  aoai13aa1n02x5               g181(.a(new_n264), .b(new_n276), .c(new_n275), .d(new_n246), .o1(new_n277));
  oai012aa1n02x5               g182(.a(new_n266), .b(new_n277), .c(new_n274), .o1(new_n278));
  aoi012aa1n02x5               g183(.a(new_n271), .b(new_n278), .c(new_n269), .o1(new_n279));
  norp02aa1n02x5               g184(.a(new_n279), .b(new_n272), .o1(\s[28] ));
  norb02aa1n02x5               g185(.a(new_n266), .b(new_n271), .out0(new_n281));
  oai012aa1n02x5               g186(.a(new_n281), .b(new_n277), .c(new_n274), .o1(new_n282));
  oao003aa1n02x5               g187(.a(\a[28] ), .b(\b[27] ), .c(new_n269), .carry(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[28] ), .b(\a[29] ), .out0(new_n284));
  aoi012aa1n02x5               g189(.a(new_n284), .b(new_n282), .c(new_n283), .o1(new_n285));
  aobi12aa1n02x5               g190(.a(new_n281), .b(new_n265), .c(new_n262), .out0(new_n286));
  nano22aa1n02x4               g191(.a(new_n286), .b(new_n283), .c(new_n284), .out0(new_n287));
  norp02aa1n02x5               g192(.a(new_n285), .b(new_n287), .o1(\s[29] ));
  xorb03aa1n02x5               g193(.a(new_n110), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g194(.a(new_n266), .b(new_n284), .c(new_n271), .out0(new_n290));
  oai012aa1n02x5               g195(.a(new_n290), .b(new_n277), .c(new_n274), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .c(new_n283), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  aoi012aa1n02x5               g198(.a(new_n293), .b(new_n291), .c(new_n292), .o1(new_n294));
  aobi12aa1n02x5               g199(.a(new_n290), .b(new_n265), .c(new_n262), .out0(new_n295));
  nano22aa1n02x4               g200(.a(new_n295), .b(new_n292), .c(new_n293), .out0(new_n296));
  norp02aa1n02x5               g201(.a(new_n294), .b(new_n296), .o1(\s[30] ));
  norb02aa1n02x5               g202(.a(new_n290), .b(new_n293), .out0(new_n298));
  aobi12aa1n02x5               g203(.a(new_n298), .b(new_n265), .c(new_n262), .out0(new_n299));
  oao003aa1n02x5               g204(.a(\a[30] ), .b(\b[29] ), .c(new_n292), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[30] ), .b(\a[31] ), .out0(new_n301));
  nano22aa1n02x4               g206(.a(new_n299), .b(new_n300), .c(new_n301), .out0(new_n302));
  oai012aa1n02x5               g207(.a(new_n298), .b(new_n277), .c(new_n274), .o1(new_n303));
  aoi012aa1n02x5               g208(.a(new_n301), .b(new_n303), .c(new_n300), .o1(new_n304));
  norp02aa1n02x5               g209(.a(new_n304), .b(new_n302), .o1(\s[31] ));
  xnbna2aa1n03x5               g210(.a(new_n111), .b(new_n105), .c(new_n106), .out0(\s[3] ));
  oaoi03aa1n02x5               g211(.a(\a[3] ), .b(\b[2] ), .c(new_n111), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g213(.a(new_n114), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g214(.a(new_n121), .b(new_n114), .out0(new_n310));
  xobna2aa1n03x5               g215(.a(new_n120), .b(new_n310), .c(new_n124), .out0(\s[6] ));
  norp02aa1n02x5               g216(.a(new_n121), .b(new_n120), .o1(new_n312));
  aoi012aa1n02x5               g217(.a(new_n125), .b(new_n114), .c(new_n312), .o1(new_n313));
  xnrb03aa1n02x5               g218(.a(new_n313), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g219(.a(\a[7] ), .b(\b[6] ), .c(new_n313), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g221(.a(new_n128), .b(new_n123), .c(new_n141), .out0(\s[9] ));
endmodule


