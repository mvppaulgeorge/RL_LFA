// Benchmark "adder" written by ABC on Thu Jul 11 11:22:21 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n180, new_n181,
    new_n182, new_n183, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n227, new_n228, new_n229, new_n230,
    new_n231, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n245, new_n246,
    new_n247, new_n248, new_n249, new_n251, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n300, new_n303, new_n305, new_n306,
    new_n308;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[6] ), .b(\a[7] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nona23aa1n02x4               g005(.a(new_n100), .b(new_n98), .c(new_n97), .d(new_n99), .out0(new_n101));
  xnrc02aa1n02x5               g006(.a(\b[5] ), .b(\a[6] ), .out0(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[4] ), .b(\a[5] ), .out0(new_n103));
  norp03aa1n02x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  160nm_ficinv00aa1n08x5       g009(.clk(\a[2] ), .clkout(new_n105));
  160nm_ficinv00aa1n08x5       g010(.clk(\b[1] ), .clkout(new_n106));
  nanp02aa1n02x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  oaoi03aa1n02x5               g012(.a(new_n105), .b(new_n106), .c(new_n107), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nona23aa1n02x4               g017(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n113));
  aoi012aa1n02x5               g018(.a(new_n109), .b(new_n111), .c(new_n110), .o1(new_n114));
  oai012aa1n02x5               g019(.a(new_n114), .b(new_n113), .c(new_n108), .o1(new_n115));
  160nm_ficinv00aa1n08x5       g020(.clk(\a[5] ), .clkout(new_n116));
  160nm_ficinv00aa1n08x5       g021(.clk(\b[4] ), .clkout(new_n117));
  nanp02aa1n02x5               g022(.a(new_n117), .b(new_n116), .o1(new_n118));
  oaoi03aa1n02x5               g023(.a(\a[6] ), .b(\b[5] ), .c(new_n118), .o1(new_n119));
  aoi012aa1n02x5               g024(.a(new_n97), .b(new_n99), .c(new_n98), .o1(new_n120));
  oaib12aa1n02x5               g025(.a(new_n120), .b(new_n101), .c(new_n119), .out0(new_n121));
  aoi012aa1n02x5               g026(.a(new_n121), .b(new_n115), .c(new_n104), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .c(new_n122), .o1(new_n123));
  xorb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g029(.a(\b[10] ), .b(\a[11] ), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[10] ), .b(\a[11] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  160nm_ficinv00aa1n08x5       g032(.clk(new_n127), .clkout(new_n128));
  norp02aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  norp02aa1n02x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  oai012aa1n02x5               g036(.a(new_n130), .b(new_n131), .c(new_n129), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[8] ), .b(\a[9] ), .o1(new_n133));
  nano23aa1n02x4               g038(.a(new_n129), .b(new_n131), .c(new_n133), .d(new_n130), .out0(new_n134));
  160nm_ficinv00aa1n08x5       g039(.clk(new_n134), .clkout(new_n135));
  oaoi13aa1n02x5               g040(.a(new_n128), .b(new_n132), .c(new_n122), .d(new_n135), .o1(new_n136));
  oai112aa1n02x5               g041(.a(new_n132), .b(new_n128), .c(new_n122), .d(new_n135), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(\s[11] ));
  norp02aa1n02x5               g043(.a(new_n136), .b(new_n125), .o1(new_n139));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  160nm_ficinv00aa1n08x5       g045(.clk(new_n140), .clkout(new_n141));
  nanp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n139), .b(new_n142), .c(new_n141), .out0(\s[12] ));
  nano23aa1n02x4               g048(.a(new_n125), .b(new_n140), .c(new_n142), .d(new_n126), .out0(new_n144));
  nanp02aa1n02x5               g049(.a(new_n144), .b(new_n134), .o1(new_n145));
  nona23aa1n02x4               g050(.a(new_n142), .b(new_n126), .c(new_n125), .d(new_n140), .out0(new_n146));
  nanp02aa1n02x5               g051(.a(new_n125), .b(new_n142), .o1(new_n147));
  oai112aa1n02x5               g052(.a(new_n147), .b(new_n141), .c(new_n146), .d(new_n132), .o1(new_n148));
  oabi12aa1n02x5               g053(.a(new_n148), .b(new_n122), .c(new_n145), .out0(new_n149));
  xorb03aa1n02x5               g054(.a(new_n149), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n151), .b(new_n149), .c(new_n152), .o1(new_n153));
  xnrb03aa1n02x5               g058(.a(new_n153), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n02x5               g059(.a(\b[14] ), .b(\a[15] ), .out0(new_n155));
  160nm_ficinv00aa1n08x5       g060(.clk(new_n155), .clkout(new_n156));
  norp02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nano23aa1n02x4               g063(.a(new_n151), .b(new_n157), .c(new_n158), .d(new_n152), .out0(new_n159));
  oai012aa1n02x5               g064(.a(new_n158), .b(new_n157), .c(new_n151), .o1(new_n160));
  160nm_ficinv00aa1n08x5       g065(.clk(new_n160), .clkout(new_n161));
  aoai13aa1n02x5               g066(.a(new_n156), .b(new_n161), .c(new_n149), .d(new_n159), .o1(new_n162));
  aoi112aa1n02x5               g067(.a(new_n156), .b(new_n161), .c(new_n149), .d(new_n159), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n162), .b(new_n163), .out0(\s[15] ));
  xnrc02aa1n02x5               g069(.a(\b[15] ), .b(\a[16] ), .out0(new_n165));
  oai112aa1n02x5               g070(.a(new_n162), .b(new_n165), .c(\b[14] ), .d(\a[15] ), .o1(new_n166));
  oaoi13aa1n02x5               g071(.a(new_n165), .b(new_n162), .c(\a[15] ), .d(\b[14] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n166), .b(new_n167), .out0(\s[16] ));
  160nm_ficinv00aa1n08x5       g073(.clk(new_n165), .clkout(new_n169));
  nano32aa1n02x4               g074(.a(new_n145), .b(new_n169), .c(new_n156), .d(new_n159), .out0(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n121), .c(new_n104), .d(new_n115), .o1(new_n171));
  nona23aa1n02x4               g076(.a(new_n158), .b(new_n152), .c(new_n151), .d(new_n157), .out0(new_n172));
  norp03aa1n02x5               g077(.a(new_n172), .b(new_n165), .c(new_n155), .o1(new_n173));
  aoi112aa1n02x5               g078(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n174));
  nona22aa1n02x4               g079(.a(new_n169), .b(new_n155), .c(new_n160), .out0(new_n175));
  oai012aa1n02x5               g080(.a(new_n175), .b(\b[15] ), .c(\a[16] ), .o1(new_n176));
  aoi112aa1n02x5               g081(.a(new_n176), .b(new_n174), .c(new_n148), .d(new_n173), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(new_n177), .b(new_n171), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g084(.clk(\a[18] ), .clkout(new_n180));
  160nm_ficinv00aa1n08x5       g085(.clk(\a[17] ), .clkout(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(\b[16] ), .clkout(new_n182));
  oaoi03aa1n02x5               g087(.a(new_n181), .b(new_n182), .c(new_n178), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[17] ), .c(new_n180), .out0(\s[18] ));
  xroi22aa1d04x5               g089(.a(new_n181), .b(\b[16] ), .c(new_n180), .d(\b[17] ), .out0(new_n185));
  nanp02aa1n02x5               g090(.a(new_n182), .b(new_n181), .o1(new_n186));
  oaoi03aa1n02x5               g091(.a(\a[18] ), .b(\b[17] ), .c(new_n186), .o1(new_n187));
  norp02aa1n02x5               g092(.a(\b[18] ), .b(\a[19] ), .o1(new_n188));
  nanp02aa1n02x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  norb02aa1n02x5               g094(.a(new_n189), .b(new_n188), .out0(new_n190));
  aoai13aa1n02x5               g095(.a(new_n190), .b(new_n187), .c(new_n178), .d(new_n185), .o1(new_n191));
  aoi112aa1n02x5               g096(.a(new_n190), .b(new_n187), .c(new_n178), .d(new_n185), .o1(new_n192));
  norb02aa1n02x5               g097(.a(new_n191), .b(new_n192), .out0(\s[19] ));
  xnrc02aa1n02x5               g098(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g099(.a(\b[19] ), .b(\a[20] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  nona22aa1n02x4               g102(.a(new_n191), .b(new_n197), .c(new_n188), .out0(new_n198));
  orn002aa1n02x5               g103(.a(\a[19] ), .b(\b[18] ), .o(new_n199));
  aobi12aa1n02x5               g104(.a(new_n197), .b(new_n191), .c(new_n199), .out0(new_n200));
  norb02aa1n02x5               g105(.a(new_n198), .b(new_n200), .out0(\s[20] ));
  nano23aa1n02x4               g106(.a(new_n188), .b(new_n195), .c(new_n196), .d(new_n189), .out0(new_n202));
  nanp02aa1n02x5               g107(.a(new_n185), .b(new_n202), .o1(new_n203));
  oai022aa1n02x5               g108(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n204));
  oaib12aa1n02x5               g109(.a(new_n204), .b(new_n180), .c(\b[17] ), .out0(new_n205));
  nona23aa1n02x4               g110(.a(new_n196), .b(new_n189), .c(new_n188), .d(new_n195), .out0(new_n206));
  oaoi03aa1n02x5               g111(.a(\a[20] ), .b(\b[19] ), .c(new_n199), .o1(new_n207));
  oabi12aa1n02x5               g112(.a(new_n207), .b(new_n206), .c(new_n205), .out0(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(new_n208), .clkout(new_n209));
  aoai13aa1n02x5               g114(.a(new_n209), .b(new_n203), .c(new_n177), .d(new_n171), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g116(.a(\b[20] ), .b(\a[21] ), .o1(new_n212));
  xorc02aa1n02x5               g117(.a(\a[21] ), .b(\b[20] ), .out0(new_n213));
  xorc02aa1n02x5               g118(.a(\a[22] ), .b(\b[21] ), .out0(new_n214));
  aoi112aa1n02x5               g119(.a(new_n212), .b(new_n214), .c(new_n210), .d(new_n213), .o1(new_n215));
  aoai13aa1n02x5               g120(.a(new_n214), .b(new_n212), .c(new_n210), .d(new_n213), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g122(.clk(\a[21] ), .clkout(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(\a[22] ), .clkout(new_n219));
  xroi22aa1d04x5               g124(.a(new_n218), .b(\b[20] ), .c(new_n219), .d(\b[21] ), .out0(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(\b[21] ), .clkout(new_n221));
  oao003aa1n02x5               g126(.a(new_n219), .b(new_n221), .c(new_n212), .carry(new_n222));
  aoi012aa1n02x5               g127(.a(new_n222), .b(new_n208), .c(new_n220), .o1(new_n223));
  nanp03aa1n02x5               g128(.a(new_n220), .b(new_n185), .c(new_n202), .o1(new_n224));
  aoai13aa1n02x5               g129(.a(new_n223), .b(new_n224), .c(new_n177), .d(new_n171), .o1(new_n225));
  xorb03aa1n02x5               g130(.a(new_n225), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g131(.a(\b[22] ), .b(\a[23] ), .o1(new_n227));
  xorc02aa1n02x5               g132(.a(\a[23] ), .b(\b[22] ), .out0(new_n228));
  xorc02aa1n02x5               g133(.a(\a[24] ), .b(\b[23] ), .out0(new_n229));
  aoi112aa1n02x5               g134(.a(new_n227), .b(new_n229), .c(new_n225), .d(new_n228), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n229), .b(new_n227), .c(new_n225), .d(new_n228), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n231), .b(new_n230), .out0(\s[24] ));
  and002aa1n02x5               g137(.a(new_n229), .b(new_n228), .o(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n233), .clkout(new_n234));
  nano32aa1n02x4               g139(.a(new_n234), .b(new_n220), .c(new_n185), .d(new_n202), .out0(new_n235));
  aoai13aa1n02x5               g140(.a(new_n220), .b(new_n207), .c(new_n202), .d(new_n187), .o1(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n222), .clkout(new_n237));
  aoi112aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n238));
  oab012aa1n02x4               g143(.a(new_n238), .b(\a[24] ), .c(\b[23] ), .out0(new_n239));
  aoai13aa1n02x5               g144(.a(new_n239), .b(new_n234), .c(new_n236), .d(new_n237), .o1(new_n240));
  xorc02aa1n02x5               g145(.a(\a[25] ), .b(\b[24] ), .out0(new_n241));
  aoai13aa1n02x5               g146(.a(new_n241), .b(new_n240), .c(new_n178), .d(new_n235), .o1(new_n242));
  aoi112aa1n02x5               g147(.a(new_n241), .b(new_n240), .c(new_n178), .d(new_n235), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n242), .b(new_n243), .out0(\s[25] ));
  norp02aa1n02x5               g149(.a(\b[24] ), .b(\a[25] ), .o1(new_n245));
  xorc02aa1n02x5               g150(.a(\a[26] ), .b(\b[25] ), .out0(new_n246));
  nona22aa1n02x4               g151(.a(new_n242), .b(new_n246), .c(new_n245), .out0(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n245), .clkout(new_n248));
  aobi12aa1n02x5               g153(.a(new_n246), .b(new_n242), .c(new_n248), .out0(new_n249));
  norb02aa1n02x5               g154(.a(new_n247), .b(new_n249), .out0(\s[26] ));
  160nm_ficinv00aa1n08x5       g155(.clk(\a[25] ), .clkout(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(\a[26] ), .clkout(new_n252));
  xroi22aa1d04x5               g157(.a(new_n251), .b(\b[24] ), .c(new_n252), .d(\b[25] ), .out0(new_n253));
  nano22aa1n02x4               g158(.a(new_n224), .b(new_n233), .c(new_n253), .out0(new_n254));
  nanp02aa1n02x5               g159(.a(new_n178), .b(new_n254), .o1(new_n255));
  oao003aa1n02x5               g160(.a(\a[26] ), .b(\b[25] ), .c(new_n248), .carry(new_n256));
  aobi12aa1n02x5               g161(.a(new_n256), .b(new_n240), .c(new_n253), .out0(new_n257));
  norp02aa1n02x5               g162(.a(\b[26] ), .b(\a[27] ), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(\b[26] ), .b(\a[27] ), .o1(new_n259));
  norb02aa1n02x5               g164(.a(new_n259), .b(new_n258), .out0(new_n260));
  xnbna2aa1n03x5               g165(.a(new_n260), .b(new_n255), .c(new_n257), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g166(.clk(new_n258), .clkout(new_n262));
  xnrc02aa1n02x5               g167(.a(\b[27] ), .b(\a[28] ), .out0(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n254), .clkout(new_n264));
  aoi012aa1n02x5               g169(.a(new_n264), .b(new_n177), .c(new_n171), .o1(new_n265));
  aoai13aa1n02x5               g170(.a(new_n233), .b(new_n222), .c(new_n208), .d(new_n220), .o1(new_n266));
  160nm_ficinv00aa1n08x5       g171(.clk(new_n253), .clkout(new_n267));
  aoai13aa1n02x5               g172(.a(new_n256), .b(new_n267), .c(new_n266), .d(new_n239), .o1(new_n268));
  oai012aa1n02x5               g173(.a(new_n259), .b(new_n268), .c(new_n265), .o1(new_n269));
  aoi012aa1n02x5               g174(.a(new_n263), .b(new_n269), .c(new_n262), .o1(new_n270));
  aoi022aa1n02x5               g175(.a(new_n255), .b(new_n257), .c(\b[26] ), .d(\a[27] ), .o1(new_n271));
  nano22aa1n02x4               g176(.a(new_n271), .b(new_n262), .c(new_n263), .out0(new_n272));
  norp02aa1n02x5               g177(.a(new_n270), .b(new_n272), .o1(\s[28] ));
  nano22aa1n02x4               g178(.a(new_n263), .b(new_n262), .c(new_n259), .out0(new_n274));
  oai012aa1n02x5               g179(.a(new_n274), .b(new_n268), .c(new_n265), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[28] ), .b(\b[27] ), .c(new_n262), .carry(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[28] ), .b(\a[29] ), .out0(new_n277));
  aoi012aa1n02x5               g182(.a(new_n277), .b(new_n275), .c(new_n276), .o1(new_n278));
  aobi12aa1n02x5               g183(.a(new_n274), .b(new_n255), .c(new_n257), .out0(new_n279));
  nano22aa1n02x4               g184(.a(new_n279), .b(new_n276), .c(new_n277), .out0(new_n280));
  norp02aa1n02x5               g185(.a(new_n278), .b(new_n280), .o1(\s[29] ));
  xorb03aa1n02x5               g186(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g187(.a(new_n260), .b(new_n277), .c(new_n263), .out0(new_n283));
  oai012aa1n02x5               g188(.a(new_n283), .b(new_n268), .c(new_n265), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[29] ), .b(\b[28] ), .c(new_n276), .carry(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[29] ), .b(\a[30] ), .out0(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n284), .c(new_n285), .o1(new_n287));
  aobi12aa1n02x5               g192(.a(new_n283), .b(new_n255), .c(new_n257), .out0(new_n288));
  nano22aa1n02x4               g193(.a(new_n288), .b(new_n285), .c(new_n286), .out0(new_n289));
  norp02aa1n02x5               g194(.a(new_n287), .b(new_n289), .o1(\s[30] ));
  xnrc02aa1n02x5               g195(.a(\b[30] ), .b(\a[31] ), .out0(new_n291));
  norb03aa1n02x5               g196(.a(new_n274), .b(new_n286), .c(new_n277), .out0(new_n292));
  aobi12aa1n02x5               g197(.a(new_n292), .b(new_n255), .c(new_n257), .out0(new_n293));
  oao003aa1n02x5               g198(.a(\a[30] ), .b(\b[29] ), .c(new_n285), .carry(new_n294));
  nano22aa1n02x4               g199(.a(new_n293), .b(new_n291), .c(new_n294), .out0(new_n295));
  oai012aa1n02x5               g200(.a(new_n292), .b(new_n268), .c(new_n265), .o1(new_n296));
  aoi012aa1n02x5               g201(.a(new_n291), .b(new_n296), .c(new_n294), .o1(new_n297));
  norp02aa1n02x5               g202(.a(new_n297), .b(new_n295), .o1(\s[31] ));
  xnrb03aa1n02x5               g203(.a(new_n108), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g204(.a(\a[3] ), .b(\b[2] ), .c(new_n108), .o1(new_n300));
  xorb03aa1n02x5               g205(.a(new_n300), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g206(.a(new_n115), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g207(.a(new_n116), .b(new_n117), .c(new_n115), .o1(new_n303));
  xnrb03aa1n02x5               g208(.a(new_n303), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norp02aa1n02x5               g209(.a(new_n103), .b(new_n102), .o1(new_n305));
  aoi012aa1n02x5               g210(.a(new_n119), .b(new_n115), .c(new_n305), .o1(new_n306));
  xnrb03aa1n02x5               g211(.a(new_n306), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g212(.a(\a[7] ), .b(\b[6] ), .c(new_n306), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g214(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


